#include "selfdrive/pandad/panda_comms.h"

#include <iterator>
#include <memory>
#include <stdexcept>

#include "common/swaglog.h"

static libusb_context *init_usb_ctx() {
  libusb_context *context = nullptr;
  int err = libusb_init(&context);
  if (err != 0) {
    LOGE("libusb initialization error");
    return nullptr;
  }

  libusb_set_option(context, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);
  return context;
}

PandaUsbHandle::PandaUsbHandle(std::string serial) {
  ssize_t num_devices;
  libusb_device **dev_list = nullptr;
  int err = 0;
  ctx = init_usb_ctx();
  if (!ctx) goto fail;

  num_devices = libusb_get_device_list(ctx, &dev_list);
  if (num_devices < 0) goto fail;

  for (ssize_t i = 0; i < num_devices; ++i) {
    libusb_device_descriptor desc;
    libusb_get_device_descriptor(dev_list[i], &desc);
    if (desc.idVendor == 0x3801 && desc.idProduct == 0xddcc) {
      int ret = libusb_open(dev_list[i], &dev_handle);
      if (dev_handle == nullptr || ret < 0) goto fail;

      unsigned char desc_serial[26] = {};
      ret = libusb_get_string_descriptor_ascii(dev_handle, desc.iSerialNumber, desc_serial, std::size(desc_serial));
      if (ret < 0) goto fail;

      hw_serial = std::string(reinterpret_cast<char *>(desc_serial), ret);
      if (serial.empty() || serial == hw_serial) break;

      libusb_close(dev_handle);
      dev_handle = nullptr;
    }
  }
  if (dev_handle == nullptr) goto fail;

  libusb_free_device_list(dev_list, 1);
  dev_list = nullptr;

  if (libusb_kernel_driver_active(dev_handle, 0) == 1) {
    libusb_detach_kernel_driver(dev_handle, 0);
  }

  err = libusb_set_configuration(dev_handle, 1);
  if (err != 0) goto fail;

  err = libusb_claim_interface(dev_handle, 0);
  if (err != 0) goto fail;
  return;

fail:
  if (dev_list != nullptr) libusb_free_device_list(dev_list, 1);
  cleanup();
  throw std::runtime_error("Error connecting to panda over USB");
}

PandaUsbHandle::~PandaUsbHandle() {
  std::lock_guard lk(hw_lock);
  cleanup();
  connected = false;
}

void PandaUsbHandle::cleanup() {
  if (dev_handle != nullptr) {
    libusb_release_interface(dev_handle, 0);
    libusb_close(dev_handle);
    dev_handle = nullptr;
  }
  if (ctx != nullptr) {
    libusb_exit(ctx);
    ctx = nullptr;
  }
}

std::vector<std::string> PandaUsbHandle::list() {
  static std::unique_ptr<libusb_context, decltype(&libusb_exit)> context(init_usb_ctx(), libusb_exit);
  std::vector<std::string> serials;
  if (!context) return serials;

  libusb_device **dev_list = nullptr;
  ssize_t num_devices = libusb_get_device_list(context.get(), &dev_list);
  if (num_devices < 0) return serials;

  for (ssize_t i = 0; i < num_devices; ++i) {
    libusb_device_descriptor desc;
    libusb_get_device_descriptor(dev_list[i], &desc);
    if (desc.idVendor == 0x3801 && desc.idProduct == 0xddcc) {
      libusb_device_handle *handle = nullptr;
      int ret = libusb_open(dev_list[i], &handle);
      if (ret < 0) continue;

      unsigned char desc_serial[26] = {};
      ret = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, desc_serial, std::size(desc_serial));
      libusb_close(handle);
      if (ret >= 0) serials.emplace_back(reinterpret_cast<char *>(desc_serial), ret);
    }
  }

  libusb_free_device_list(dev_list, 1);
  return serials;
}

void PandaUsbHandle::handle_usb_issue(int err, const char func[]) {
  LOGE_100("usb error %d \"%s\" in %s", err, libusb_strerror(static_cast<libusb_error>(err)), func);
  if (err == LIBUSB_ERROR_NO_DEVICE) {
    LOGE("lost USB connection to panda");
    connected = false;
  }
}

int PandaUsbHandle::control_write(uint8_t request, uint16_t param1, uint16_t param2, unsigned int timeout) {
  if (!connected) return LIBUSB_ERROR_NO_DEVICE;

  std::lock_guard lk(hw_lock);
  int err;
  do {
    err = libusb_control_transfer(dev_handle, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                                  request, param1, param2, nullptr, 0, timeout);
    if (err < 0) handle_usb_issue(err, __func__);
  } while (err < 0 && connected);
  return err;
}

int PandaUsbHandle::control_read(uint8_t request, uint16_t param1, uint16_t param2, unsigned char *data, uint16_t length,
                                 unsigned int timeout) {
  if (!connected) return LIBUSB_ERROR_NO_DEVICE;

  std::lock_guard lk(hw_lock);
  int err;
  do {
    err = libusb_control_transfer(dev_handle, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                                  request, param1, param2, data, length, timeout);
    if (err < 0) handle_usb_issue(err, __func__);
  } while (err < 0 && connected);
  return err;
}

int PandaUsbHandle::bulk_write(unsigned char endpoint, unsigned char *data, int length, unsigned int timeout) {
  if (!connected) return 0;

  std::lock_guard lk(hw_lock);
  int err;
  int transferred = 0;
  do {
    err = libusb_bulk_transfer(dev_handle, endpoint, data, length, &transferred, timeout);
    if (err == LIBUSB_ERROR_TIMEOUT) {
      LOGW("Panda transmit buffer full");
      break;
    } else if (err != 0 || length != transferred) {
      handle_usb_issue(err, __func__);
    }
  } while (err != 0 && connected);
  return transferred;
}

int PandaUsbHandle::bulk_read(unsigned char endpoint, unsigned char *data, int length, unsigned int timeout) {
  if (!connected) return 0;

  std::lock_guard lk(hw_lock);
  int err;
  int transferred = 0;
  do {
    err = libusb_bulk_transfer(dev_handle, endpoint, data, length, &transferred, timeout);
    if (err == LIBUSB_ERROR_TIMEOUT) {
      break;
    } else if (err == LIBUSB_ERROR_OVERFLOW) {
      comms_healthy = false;
      LOGE_100("Panda USB overflow, received 0x%x bytes", transferred);
    } else if (err != 0) {
      handle_usb_issue(err, __func__);
    }
  } while (err != 0 && connected);
  return transferred;
}
