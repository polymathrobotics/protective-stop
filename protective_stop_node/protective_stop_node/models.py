from pydantic import BaseModel, Field, validator
from pydantic import validator, root_validator
from pydantic.json import pydantic_encoder
from rclpy.time import Time
from typing import Optional, List, TypeVar, Generic, Dict

from protective_stop_msg.msg import (
    ProtectiveStop,
    ProtectiveStopStatus,
    ProtectiveStopParams,
    ProtectiveStopDebugRemote,
)

from enum import Enum
from builtin_interfaces.msg import Time as TimeMsg


class Ros2Time(BaseModel):
    sec: int = Field(..., ge=0, description="Seconds since epoch")
    nanosec: int = Field(..., ge=0, lt=1_000_000_000, description="Nanoseconds component")

    @classmethod
    def from_ros_time(cls, ros_time: Time) -> "Ros2Time":
        """
        Convert an `rclpy.time.Time` object to `Ros2Time`.
        """
        return cls(sec=ros_time.seconds_nanoseconds()[0], nanosec=ros_time.seconds_nanoseconds()[1])

    @classmethod
    def from_time_msg(cls, time_msg: TimeMsg) -> "Ros2Time":
        """
        Convert a `builtin_interfaces.msg.Time` message to `Ros2Time`.
        """
        return cls(sec=time_msg.sec, nanosec=time_msg.nanosec)

    def to_ros_time(self, clock) -> Time:
        """
        Convert `Ros2Time` to an `rclpy.time.Time` object.
        """
        return Time(seconds=self.sec, nanoseconds=self.nanosec, clock_type=clock.clock_type)

    def to_time_msg(self) -> TimeMsg:
        """
        Convert `Ros2Time` to a `builtin_interfaces.msg.Time` message.
        """
        return TimeMsg(sec=self.sec, nanosec=self.nanosec)


class PStopRemoteStatusEnum(Enum):
    ACTIVE = ProtectiveStopStatus.ACTIVE
    DEACTIVATED = ProtectiveStopStatus.DEACTIVATED
    UNSTABLE = ProtectiveStopStatus.UNSTABLE


class ConnectionStatus(BaseModel):
    status: PStopRemoteStatusEnum = Field(PStopRemoteStatusEnum.ACTIVE)
    message: str = Field("")

    @validator("status")
    def check_status_type(cls, value):
        assert isinstance(value, PStopRemoteStatusEnum), (
            "status must be of type PStopRemoteStatusEnum"
        )
        return value

    def to_ros_message(self) -> ProtectiveStopStatus:
        return ProtectiveStopStatus(
            status=self.status.value,
            message=self.message,
        )


class PStopModel(BaseModel):
    pstop_pressed: bool = Field(True)  # Default to True to prevent accidental activation
    connection_status: ConnectionStatus = Field(ConnectionStatus())
    counter: int = Field(0, ge=0)
    init_timestamp: Ros2Time = Field(...)  # init_timestamp
    receive_timestamp: Optional[Ros2Time] = Field(...)  # receive_timestamp
    remote_timestamp: Optional[Ros2Time] = Field(...)  # remote_timestamp

    @root_validator(pre=True)
    def convert_ros_time(cls, values):
        """
        Converts `rclpy.time.Time` or `builtin_interfaces.msg.Time`
        into `Ros2Time` before validation.
        """
        if isinstance(values.get("ros_time"), Time):
            values["ros_time"] = Ros2Time.from_ros_time(values["ros_time"])
        elif isinstance(values.get("ros_time"), TimeMsg):
            values["ros_time"] = Ros2Time.from_time_msg(values["ros_time"])
        return values

    # Convert into ros message
    def to_ros_message(self, uuid: str) -> ProtectiveStopDebugRemote:
        return ProtectiveStopDebugRemote(
            uuid=uuid,
            pstop_pressed=self.pstop_pressed,
            connection_status=self.connection_status.to_ros_message(),
            counter=self.counter,
            init_timestamp=self.init_timestamp.to_time_msg(),
            receive_timestamp=self.receive_timestamp.to_time_msg()
            if self.receive_timestamp
            else None,
        )
