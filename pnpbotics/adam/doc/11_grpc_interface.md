# gRPC接口说明

> Source: https://wiki.pndbotics.com/robot/grpc_interface/

上层运动控制开发基于PNDbotics机器人控制系统开发，通过调用底层运动控制接口可实现上层运动控制功能，也可基于开放接口及内置运动规划控制算法进行个性化开发。上层运动控制实现方式为grpc，用户可根据例程自定义客户端调用相关进行机器人控制及开发，服务端暂不支持二次开发。

## 接口定义文件 proto

```protobuf
syntax = "proto3";

package adam_control;

// Define services
service RobotControl {
  rpc SetMode (SetModeRequest) returns (SetModeResponse);
  rpc SetStandMotion (SetStandMotionRequest) returns (SetStandMotionResponse);
  rpc SetStandCarryBox (SetCarryBoxRequest) returns (SetCarryBoxResponse);
  rpc SetStandAction (SetActionRequest) returns (SetActionResponse);
  rpc SetStandDynamic (SetDynamicStandRequest) returns (SetDynamicStandResponse);
  rpc SetSpeed (SetSpeedRequest) returns (SetSpeedResponse);
  rpc AutoUnigaitCOM (SetUnigaitCOMRequest) returns (SetUnigaitCOMResponse);
  rpc SetErrorClear (SetErrorClearRequest) returns (SetErrorClearResponse);
  rpc GetStandList (GetStandListRequest) returns (GetStandListResponse);
  rpc GetRobotState (GetRobotStateRequest) returns (GetRobotStateResponse);
}
```

## 客户端示例

### 依赖安装及编译

下载 [client示例工程](https://wiki.pndbotics.com/assets/sdk/pndbotics_grpc_client_v1.0.0.zip)

#### 克隆 gRPC 仓库

```bash
git clone -b v1.46.3 https://github.com/grpc/grpc
cd grpc
git submodule update --init

# python install
pip install grpcio-tools
```

#### 构建和安装 gRPC

```bash
mkdir -p cmake/build
cd cmake/build
cmake ../..
make -j$(nproc)
sudo make install
```

#### 安装 Protocol Buffers

```bash
sudo apt-get install -y protobuf-compiler
sudo apt-get install -y libprotobuf-dev
```

#### 安装 nlohmann json

```bash
sudo apt-get install nlohmann-json3-dev
```

### Client说明

Client包为客户端示例工程，其中：
- `proto` 文件夹中包含定义完毕的运控端控制相关功能的协议文件
- `include` 文件夹中包含使用protobuf根据.proto文件生成的grpc相关的源文件以及头文件，后续可直接根据生成的api接口进行复写以及调用
- `src` 文件夹中为 C++ 客户端示例代码
- `python` 文件夹中为 python 客户端示例代码

### 客户端IP

在连接机器人服务端之前，请先修改 `Client/ip_config.json` 中的服务端ip，即对应的机器人主机ip。(C++ 版本客户端请在修改ip之后请重新编译)

> **注意**: 由于服务端端口号固定为 `6666`，所以请勿修改客户端中端口号。

```json
{
  "server": {
    "ip": "xx.xx.xx.xx"
  }
}
```

### 客户端编译运行

客户端下有三个脚本文件：
- `build.sh`: grpc代码生成与 C++ 客户端编译
- `run.sh`: 客户端运行，可选择 C++ 或 Python 版本
- `clean.sh`: 清理编译文件

```bash
###### Compile & Build ######
cd Client
chmod +x build.sh
./build.sh

###### Run ######
chmod +x run.sh
./run.sh
# When using the run.sh script to start a client, add parameters to select the type:
# Example: ./run.sh bin && ./run.sh python

###### Clean ######
chmod +x clean.sh
./clean.sh
```

### 客户端代码&接口调用说明

`Client` 包中给出了 python 以及 C++ 两种客户端实现方式，使用proto文件中定义的接口实现对应的控制功能，用户可参考客户端示例代码进行客户端或者调用api的复写。
