/*
 * usb_fan_hwmon.c - Linux Kernel HID Driver for 32U4 Fan Controller
 *
 * Supports:
 *  - HWMON interface (fan1-4_input, pwm1-4)
 *  - USB HID Raw Interaction
 *
 * Security Note:
 *  - Uses devm_* managed resources to prevent leaks/UAF on disconnect.
 *  - Validates all buffer bounds.
 *  - Uses mutex for protocol synchronization.
 */

#include <linux/module.h>
#include <linux/hid.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/slab.h>

#define DRIVER_VERSION "0.1"
#define DRIVER_AUTHOR "PowerLi"
#define DRIVER_DESC "32U4 Fan Controller HWMON Driver"

#define USB_VENDOR_ID   0x1B4F
#define USB_PRODUCT_ID  0x9206

#define CMD_FAN_SELECT      0x10
#define CMD_FAN_FIXED_PWM   0x13
#define CMD_FAN_READ_RPM    0x16

#define OP_WRITE_ONE        0x06
#define OP_READ_TWO         0x09

#define FAN_COUNT 4
#define REPORT_SIZE 64

struct fan_device {
    struct hid_device *hdev;
    struct device *hwmon_dev;
    struct mutex lock; // Protects buffer access and sequence
    u8 *transfer_buffer; // DMA-safe buffer for HID transfers
    struct hid_report *out_report; /* Cache Output Report */

    u16 rpm_cache[FAN_COUNT];
    unsigned long last_rpm_update; // Jiffies
    u8 pwm_cache[FAN_COUNT];
};

// Protocol Helper: Send Command
static int fan_send_command(struct fan_device *data, u8 op, u8 cmd, u8 *params, u8 param_len)
{
    int i;
    u8 seq = 1; // Simplified sequence

    // Bounds check
    if (param_len > 60) return -EINVAL;
    if (!data->out_report) return -ENODEV;

    // Use transfer_buffer to construct packet locally first
    data->transfer_buffer[0] = 3 + param_len; // Len
    data->transfer_buffer[1] = seq;
    data->transfer_buffer[2] = op;
    data->transfer_buffer[3] = cmd;

    // Clear rest
    memset(&data->transfer_buffer[4], 0, REPORT_SIZE - 4);

    if (param_len > 0 && params) {
        memcpy(&data->transfer_buffer[4], params, param_len);
    }
    
    // Copy to Report Fields
    // RawHID Report usually has 1 field with count=64 (Byte array)
    // or 64 fields of count=1?
    // Usually standard Arduino RawHID: Usage Page 0xFFAB, Usage 0x0200.
    // It has one Main item with ReportCount 64, ReportSize 8.
    // So field[0]->report_count = 64. value[] array has 64 entries.
    
    if (data->out_report->maxfield < 1) return -ENODEV;
    
    for (i = 0; i < REPORT_SIZE; i++) {
        // Safe access to value array
        if (i < data->out_report->field[0]->report_count) {
             data->out_report->field[0]->value[i] = data->transfer_buffer[i];
        }
    }

    // Submit Request (Async or Sync? standard request is async used by raw_event)
    // HID_REQ_SET_REPORT triggers Output Report (via Int or Ctrl)
    hid_hw_request(data->hdev, data->out_report, HID_REQ_SET_REPORT);

    // Note: hid_hw_request is void (fire and forget for output usually).
    // If we need blocking wait (for response), `hid_hw_wait`? No.
    // We assume reliability.
    
    return 0;
}



// HWMON Attributes
static ssize_t pwm_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct fan_device *data = dev_get_drvdata(dev);
    int index = to_sensor_dev_attr(attr)->index;
    return sprintf(buf, "%d\n", data->pwm_cache[index]);
}

static ssize_t pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct fan_device *data = dev_get_drvdata(dev);
    int index = to_sensor_dev_attr(attr)->index;
    long val;
    int ret;
    u8 pwm_val;
    u8 params[1];

    if (kstrtol(buf, 10, &val) < 0)
        return -EINVAL;

    if (val < 0 || val > 255)
        return -EINVAL;

    pwm_val = (u8)val;
    
    mutex_lock(&data->lock);
    
    // Select Fan
    params[0] = index;
    ret = fan_send_command(data, OP_WRITE_ONE, CMD_FAN_SELECT, params, 1);
    if (ret < 0) {
        mutex_unlock(&data->lock);
        return ret;
    }
    
    // Set PWM
    params[0] = pwm_val;
    ret = fan_send_command(data, OP_WRITE_ONE, CMD_FAN_FIXED_PWM, params, 1);
    if (ret < 0) {
        mutex_unlock(&data->lock);
        return ret;
    }

    data->pwm_cache[index] = pwm_val;
    mutex_unlock(&data->lock);

    return count;
}

static ssize_t fan_input_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct fan_device *data = dev_get_drvdata(dev);
    int index = to_sensor_dev_attr(attr)->index;
    
    return sprintf(buf, "%d\n", data->rpm_cache[index]);
}

// Workqueue for Polling RPM
static void fan_poll_work_handler(struct work_struct *work);
static DECLARE_DELAYED_WORK(fan_poll_work, fan_poll_work_handler);
static struct fan_device *poll_data = NULL; // Global for simple worker access (assuming single device)



// Define Sensors
static SENSOR_DEVICE_ATTR(fan1_input, 0444, fan_input_show, NULL, 0);
static SENSOR_DEVICE_ATTR(fan2_input, 0444, fan_input_show, NULL, 1);
static SENSOR_DEVICE_ATTR(fan3_input, 0444, fan_input_show, NULL, 2);
static SENSOR_DEVICE_ATTR(fan4_input, 0444, fan_input_show, NULL, 3);

static SENSOR_DEVICE_ATTR(pwm1, 0644, pwm_show, pwm_store, 0);
static SENSOR_DEVICE_ATTR(pwm2, 0644, pwm_show, pwm_store, 1);
static SENSOR_DEVICE_ATTR(pwm3, 0644, pwm_show, pwm_store, 2);
static SENSOR_DEVICE_ATTR(pwm4, 0644, pwm_show, pwm_store, 3);

static struct attribute *fan_attrs[] = {
    &sensor_dev_attr_fan1_input.dev_attr.attr,
    &sensor_dev_attr_fan2_input.dev_attr.attr,
    &sensor_dev_attr_fan3_input.dev_attr.attr,
    &sensor_dev_attr_fan4_input.dev_attr.attr,
    &sensor_dev_attr_pwm1.dev_attr.attr,
    &sensor_dev_attr_pwm2.dev_attr.attr,
    &sensor_dev_attr_pwm3.dev_attr.attr,
    &sensor_dev_attr_pwm4.dev_attr.attr,
    NULL
};

ATTRIBUTE_GROUPS(fan);

// Polling Worker Implementation
static void fan_poll_work_handler(struct work_struct *work)
{
    int ret;
    int i;
    
    // Simple logic: Trigger Read for all fans.
    // Since we don't block here waiting for reply, we just fire commands.
    // The reply comes in raw_event logic.
    
    if (!poll_data) return;

    mutex_lock(&poll_data->lock);
    
    for (i = 0; i < FAN_COUNT; i++) {
        // We actually only need to send READ request if we TRUST the firmware 
        // to return the correct fan ID.
        // Our updated generic FW returns Fan Index.
        
        // 1. Select Fan? (Protocol requires Select first?)
        // The simple protocol: OP_WRITE_ONE + CMD_FAN_SELECT
        // Then OP_READ_TWO + CMD_FAN_READ_RPM
        
        // Let's do it safely: Select then Read.
        u8 params[1];
        params[0] = i;
        ret = fan_send_command(poll_data, OP_WRITE_ONE, CMD_FAN_SELECT, params, 1);
        if (ret < 0) continue;
        
        ret = fan_send_command(poll_data, OP_READ_TWO, CMD_FAN_READ_RPM, NULL, 0);
        if (ret < 0) continue;
    }
    
    mutex_unlock(&poll_data->lock);
    
    schedule_delayed_work(&fan_poll_work, HZ / 2); // 2Hz
}

// Improved raw_event with Polling Context
static int fan_raw_event_improved(struct hid_device *hdev, struct hid_report *report, u8 *data, int size)
{
    struct fan_device *fan = hid_get_drvdata(hdev);
    
    if (size < 4) return 0;
    
    // Check for Read RPM Response
    // Format: [Len, Seq, Op, Cmd, FanID, RPMLo, RPMHi]
    // Op=0x09 (READ_TWO), Cmd=0x16 (READ_RPM)
    
    // HID-Project RawHID packet usually starts at data[0] = Length?
    // Let's assume standard packet structure we defined.
    
    u8 op = data[2];
    u8 cmd = data[3];
    
    if (op == OP_READ_TWO && cmd == CMD_FAN_READ_RPM) {
        if (size >= 7) {
            u8 fan_idx = data[4];
            u16 rpm = data[5] | (data[6] << 8);
            
            if (fan_idx < FAN_COUNT) {
                fan->rpm_cache[fan_idx] = rpm;
            }
        }
    }
    
    return 0;
}


static int fan_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
    struct fan_device *data;
    int ret;

    // Use managed allocation (devm_kzalloc) -> Frees automatically on disconnect
    data = devm_kzalloc(&hdev->dev, sizeof(struct fan_device), GFP_KERNEL);
    if (!data) return -ENOMEM;

    data->hdev = hdev;
    data->transfer_buffer = devm_kzalloc(&hdev->dev, REPORT_SIZE, GFP_KERNEL);
    if (!data->transfer_buffer) return -ENOMEM;

    mutex_init(&data->lock);
    hid_set_drvdata(hdev, data);

    ret = hid_parse(hdev);
    if (ret) {
        hid_err(hdev, "parse failed\n");
        return ret;
    }

    ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
    if (ret) {
        hid_err(hdev, "hw start failed\n");
        return ret;
    }
    
    // Open the device to ensure Interrupt IN URBs are submitted
    // forcing the kernel to listen for responses.
    ret = hid_hw_open(hdev);
    if (ret) {
        hid_err(hdev, "hw open failed\n");
        hid_hw_stop(hdev);
        return ret;
    }
    
    // Register HWMON
    data->hwmon_dev = devm_hwmon_device_register_with_groups(&hdev->dev, "usb_fan_hwmon", data, fan_groups);
    if (IS_ERR(data->hwmon_dev)) {
        ret = PTR_ERR(data->hwmon_dev);
        hid_hw_close(hdev);
        hid_hw_stop(hdev);
        return ret;
    }
    
    // Setup Global Poll Data (for simple worker access)
    poll_data = data;

    // Find Output Report
    // RawHID devices typically have one Output Report.
    // We cached it for use in send_command.
    struct list_head *head = &hdev->report_enum[HID_OUTPUT_REPORT].report_list;
    if (!list_empty(head)) {
        poll_data->out_report = list_first_entry(head, struct hid_report, list);
        dev_info(&hdev->dev, "Found Output Report ID: %d\n", poll_data->out_report->id);
    } else {
        dev_warn(&hdev->dev, "No Output Report found! Transfers may fail.\n");
    }

    // Start Polling Work
    poll_data->last_rpm_update = jiffies - HZ; // Force immediate update
    schedule_delayed_work(&fan_poll_work, 0);

    dev_info(&hdev->dev, "Fan Controller Probed Successfully\n");

    return 0;
}

static void fan_remove(struct hid_device *hdev)
{
    cancel_delayed_work_sync(&fan_poll_work);
    hid_hw_close(hdev);
    hid_hw_stop(hdev);
}


static const struct hid_device_id fan_devices[] = {
    { HID_USB_DEVICE(USB_VENDOR_ID, USB_PRODUCT_ID) },
    { }
};
MODULE_DEVICE_TABLE(hid, fan_devices);

static struct hid_driver fan_driver = {
    .name = "usb_fan_hwmon",
    .id_table = fan_devices,
    .probe = fan_probe,
    .remove = fan_remove,
    .raw_event = fan_raw_event_improved,
};

module_hid_driver(fan_driver);

MODULE_LICENSE("GPL");
