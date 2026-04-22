import math
import time
from typing import Optional, Sequence

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float64

try:
    from smbus2 import SMBus
except ImportError:  # pragma: no cover - hardware dependency
    try:
        from smbus import SMBus  # type: ignore
    except ImportError:  # pragma: no cover - hardware dependency
        SMBus = None


class BNO055:
    REG_CHIP_ID = 0x00
    REG_ACC_DATA_X_LSB = 0x08
    REG_MAG_DATA_X_LSB = 0x0E
    REG_GYR_DATA_X_LSB = 0x14
    REG_QUA_DATA_W_LSB = 0x20
    REG_LIA_DATA_X_LSB = 0x28
    REG_TEMP = 0x34
    REG_CALIB_STAT = 0x35
    REG_OPR_MODE = 0x3D
    REG_PWR_MODE = 0x3E
    REG_SYS_TRIGGER = 0x3F

    MODE_CONFIG = 0x00
    MODE_IMU = 0x08
    MODE_NDOF = 0x0C
    CHIP_ID = 0xA0

    def __init__(self, bus_id: int, address: int):
        if SMBus is None:
            raise RuntimeError('Neither smbus2 nor smbus is installed.')
        self.bus = SMBus(bus_id)
        self.address = address
        self.initialized = False

    def close(self):
        self.bus.close()

    def initialize(self, mode: int) -> bool:
        chip_id = self._read_byte(self.REG_CHIP_ID)
        if chip_id != self.CHIP_ID:
            return False

        self._write_byte(self.REG_OPR_MODE, self.MODE_CONFIG)
        time.sleep(0.025)
        self._write_byte(self.REG_SYS_TRIGGER, 0x20)
        time.sleep(0.7)

        for _ in range(10):
            chip_id = self._read_byte(self.REG_CHIP_ID)
            if chip_id == self.CHIP_ID:
                break
            time.sleep(0.1)
        if chip_id != self.CHIP_ID:
            return False

        self._write_byte(self.REG_PWR_MODE, 0x00)
        time.sleep(0.01)
        self._write_byte(self.REG_OPR_MODE, mode)
        time.sleep(0.025)
        self.initialized = True
        return True

    def read_quaternion(self):
        data = self._read_block(self.REG_QUA_DATA_W_LSB, 8)
        scale = 1.0 / 16384.0
        return tuple(self._s16(data[i], data[i + 1]) * scale for i in range(0, 8, 2))

    def read_gyroscope(self):
        data = self._read_block(self.REG_GYR_DATA_X_LSB, 6)
        scale = (1.0 / 16.0) * math.pi / 180.0
        return tuple(self._s16(data[i], data[i + 1]) * scale for i in range(0, 6, 2))

    def read_linear_acceleration(self):
        data = self._read_block(self.REG_LIA_DATA_X_LSB, 6)
        scale = 0.01
        return tuple(self._s16(data[i], data[i + 1]) * scale for i in range(0, 6, 2))

    def read_accelerometer(self):
        data = self._read_block(self.REG_ACC_DATA_X_LSB, 6)
        scale = 0.01
        return tuple(self._s16(data[i], data[i + 1]) * scale for i in range(0, 6, 2))

    def read_magnetometer(self):
        data = self._read_block(self.REG_MAG_DATA_X_LSB, 6)
        scale = 1.0 / 16.0
        return tuple(self._s16(data[i], data[i + 1]) * scale for i in range(0, 6, 2))

    def read_temperature(self):
        raw = self._read_byte(self.REG_TEMP)
        return raw - 256 if raw > 127 else raw

    def read_calibration(self):
        value = self._read_byte(self.REG_CALIB_STAT)
        return ((value >> 6) & 0x03, (value >> 4) & 0x03, (value >> 2) & 0x03, value & 0x03)

    def _write_byte(self, reg: int, value: int):
        self.bus.write_byte_data(self.address, reg, value)

    def _read_byte(self, reg: int):
        return self.bus.read_byte_data(self.address, reg)

    def _read_block(self, reg: int, length: int):
        return self.bus.read_i2c_block_data(self.address, reg, length)

    @staticmethod
    def _s16(lsb: int, msb: int):
        value = (msb << 8) | lsb
        return value - 65536 if value & 0x8000 else value


class BNO055Node(Node):
    def __init__(self):
        super().__init__('bno055_node')

        self.declare_parameter('i2c_bus', '/dev/i2c-1')
        self.declare_parameter('i2c_address', 0x28)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_frequency', 50.0)
        self.declare_parameter('operation_mode', 0x0C)
        self.declare_parameter('orientation_covariance', [0.0025, 0.0025, 0.0025])
        self.declare_parameter('angular_velocity_covariance', [0.0004, 0.0004, 0.0004])
        self.declare_parameter('linear_acceleration_covariance', [0.0064, 0.0064, 0.0064])

        bus_path = str(self.get_parameter('i2c_bus').value)
        bus_id = int(bus_path.split('-')[-1]) if bus_path.startswith('/dev/i2c-') else int(bus_path)
        address = int(self.get_parameter('i2c_address').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        mode = int(self.get_parameter('operation_mode').value)

        self.driver: Optional[BNO055] = None
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)
        self.temp_pub = self.create_publisher(Float64, '/imu/temp', 10)

        try:
            self.driver = BNO055(bus_id, address)
            for attempt in range(1, 6):
                self.get_logger().info(
                    f'Initializing BNO055 attempt={attempt}/5 bus={bus_path} addr=0x{address:02x}'
                )
                if self.driver.initialize(mode):
                    break
                time.sleep(2.0)
            else:
                self.get_logger().error('BNO055 initialization failed after 5 attempts.')
        except Exception as exc:  # pragma: no cover - hardware dependency
            self.driver = None
            self.get_logger().error(f'BNO055 setup failed: {exc}')

        freq = float(self.get_parameter('publish_frequency').value)
        self.create_timer(1.0 / max(freq, 1.0), self._publish)
        self.create_timer(5.0, self._log_calibration)

    def _diag3(self, values: Sequence[float], default: float):
        data = [0.0] * 9
        diag = list(values[:3]) if len(values) >= 3 else [default, default, default]
        data[0] = float(diag[0])
        data[4] = float(diag[1])
        data[8] = float(diag[2])
        return data

    def _publish(self):
        if self.driver is None or not self.driver.initialized:
            return

        try:
            imu = Imu()
            imu.header.stamp = self.get_clock().now().to_msg()
            imu.header.frame_id = self.frame_id

            ori_cov = self.get_parameter('orientation_covariance').value
            gyro_cov = self.get_parameter('angular_velocity_covariance').value
            acc_cov = self.get_parameter('linear_acceleration_covariance').value

            qw, qx, qy, qz = self.driver.read_quaternion()
            imu.orientation.w = qw
            imu.orientation.x = qx
            imu.orientation.y = qy
            imu.orientation.z = qz
            imu.orientation_covariance = self._diag3(ori_cov, 0.0025)

            gx, gy, gz = self.driver.read_gyroscope()
            imu.angular_velocity.x = gx
            imu.angular_velocity.y = gy
            imu.angular_velocity.z = gz
            imu.angular_velocity_covariance = self._diag3(gyro_cov, 0.0004)

            try:
                ax, ay, az = self.driver.read_linear_acceleration()
            except Exception:
                ax, ay, az = self.driver.read_accelerometer()
            imu.linear_acceleration.x = ax
            imu.linear_acceleration.y = ay
            imu.linear_acceleration.z = az
            imu.linear_acceleration_covariance = self._diag3(acc_cov, 0.0064)
            self.imu_pub.publish(imu)

            mx, my, mz = self.driver.read_magnetometer()
            mag = MagneticField()
            mag.header = imu.header
            mag.magnetic_field.x = mx * 1e-6
            mag.magnetic_field.y = my * 1e-6
            mag.magnetic_field.z = mz * 1e-6
            self.mag_pub.publish(mag)

            temp = Float64()
            temp.data = float(self.driver.read_temperature())
            self.temp_pub.publish(temp)
        except Exception as exc:  # pragma: no cover - hardware dependency
            self.get_logger().warn(f'BNO055 read failed: {exc}')

    def _log_calibration(self):
        if self.driver is None or not self.driver.initialized:
            return
        try:
            sys, gyro, accel, mag = self.driver.read_calibration()
            self.get_logger().info(
                f'BNO055 calibration sys={sys} gyro={gyro} accel={accel} mag={mag}'
            )
        except Exception as exc:  # pragma: no cover - hardware dependency
            self.get_logger().warn(f'Read calibration failed: {exc}')

    def destroy_node(self):
        if self.driver is not None:
            try:
                self.driver.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BNO055Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
