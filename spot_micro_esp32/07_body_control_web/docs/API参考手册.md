# API参考文档

## 📡 接口概览

ESP32四足机器人网页实时控制系统提供两种通信协议：HTTP RESTful API和WebSocket实时通信。系统优先使用WebSocket进行实时控制，HTTP作为降级备用方案。

## 🌐 HTTP RESTful API

### 基础信息

- **服务器地址**: `http://192.168.1.7:12580`
- **协议版本**: HTTP/1.1
- **响应格式**: JSON
- **字符编码**: UTF-8

### 通用响应格式

```json
{
  "status": "ok|error",
  "message": "描述信息（可选）",
  "data": "响应数据（可选）"
}
```

### 状态码说明

| HTTP状态码 | 含义 | 说明 |
|-----------|------|------|
| 200 | OK | 请求成功 |
| 400 | Bad Request | 请求参数错误 |
| 500 | Internal Server Error | 服务器内部错误 |

---

## 🏠 主页接口

### GET /

获取网页控制界面。

**请求示例**:
```http
GET / HTTP/1.1
Host: 192.168.1.7:12580
```

**响应**: HTML页面内容

---

## 📍 位置控制接口

### GET /api/pos/x

控制机器人X轴位置。

**参数**:
- `value` (required): 位置值，范围 [-100, 100]，单位：mm

**请求示例**:
```http
GET /api/pos/x?value=50 HTTP/1.1
Host: 192.168.1.7:12580
```

**响应示例**:
```json
{
  "status": "ok"
}
```

**参数映射**:
- 网页值 [-100, 100] mm → 机器人坐标 [-0.100, 0.100] m

### GET /api/pos/y

控制机器人Y轴位置。

**参数**:
- `value` (required): 位置值，范围 [-100, 100]，单位：mm

**请求示例**:
```http
GET /api/pos/y?value=-30 HTTP/1.1
Host: 192.168.1.7:12580
```

**响应示例**:
```json
{
  "status": "ok"
}
```

### GET /api/pos/z

控制机器人Z轴位置。

**参数**:
- `value` (required): 位置值，范围 [-100, 100]，单位：mm

**请求示例**:
```http
GET /api/pos/z?value=-50 HTTP/1.1
Host: 192.168.1.7:12580
```

**响应示例**:
```json
{
  "status": "ok"
}
```

**参数映射**:
- 网页值 [-100, 100] mm → 机器人坐标 [-0.200, 0.000] m（相对于-0.10m基准）

---

## 🎯 姿态控制接口

### GET /api/rot/roll

控制机器人Roll翻滚角度。

**参数**:
- `value` (required): 角度值，范围 [-180, 180]，单位：度

**请求示例**:
```http
GET /api/rot/roll?value=15 HTTP/1.1
Host: 192.168.1.7:12580
```

**响应示例**:
```json
{
  "status": "ok"
}
```

### GET /api/rot/pitch

控制机器人Pitch俯仰角度。

**参数**:
- `value` (required): 角度值，范围 [-180, 180]，单位：度

**请求示例**:
```http
GET /api/rot/pitch?value=-10 HTTP/1.1
Host: 192.168.1.7:12580
```

**响应示例**:
```json
{
  "status": "ok"
}
```

### GET /api/rot/yaw

控制机器人Yaw偏航角度。

**参数**:
- `value` (required): 角度值，范围 [-180, 180]，单位：度

**请求示例**:
```http
GET /api/rot/yaw?value=45 HTTP/1.1
Host: 192.168.1.7:12580
```

**响应示例**:
```json
{
  "status": "ok"
}
```

---

## 🎮 动作控制接口

### GET /api/action/1

执行圆锥动作。

**参数**:
- `beta` (optional): 圆锥角度，范围 [5, 45]，单位：度，默认值：30

**请求示例**:
```http
GET /api/action/1?beta=25 HTTP/1.1
Host: 192.168.1.7:12580
```

**响应示例**:
```json
{
  "status": "ok"
}
```

**动作说明**:
- 机器人执行圆锥形运动轨迹
- beta参数控制圆锥的张开角度
- 动作持续时间约10-15秒

---

## 🎭 预设姿势接口

### GET /api/preset/0

切换到趴下姿势。

**请求示例**:
```http
GET /api/preset/0 HTTP/1.1
Host: 192.168.1.7:12580
```

**响应示例**:
```json
{
  "status": "ok"
}
```

**姿势说明**:
- 机器人切换到趴下休息姿势
- 所有关节移动到安全位置
- 适合长时间待机

### GET /api/preset/1

切换到站立姿势。

**请求示例**:
```http
GET /api/preset/1 HTTP/1.1
Host: 192.168.1.7:12580
```

**响应示例**:
```json
{
  "status": "ok"
}
```

**姿势说明**:
- 机器人切换到标准站立姿势
- 准备执行其他动作
- 默认的工作姿势

---

## 🚨 紧急控制接口

### GET /api/emergency/stop

紧急停止所有运动。

**请求示例**:
```http
GET /api/emergency/stop HTTP/1.1
Host: 192.168.1.7:12580
```

**响应示例**:
```json
{
  "status": "ok"
}
```

**功能说明**:
- 立即停止所有正在执行的动作
- 保持当前位置不动
- 清除所有运动队列
- 用于紧急情况处理

---

## 🔗 WebSocket实时通信

### 连接信息

- **端点地址**: `ws://192.168.1.7:12580/ws`
- **协议**: WebSocket (RFC 6455)
- **子协议**: 无
- **心跳间隔**: 30秒

### 连接建立

```javascript
const ws = new WebSocket('ws://192.168.1.7:12580/ws');

ws.onopen = function(event) {
    console.log('WebSocket连接成功');
};

ws.onclose = function(event) {
    console.log('WebSocket连接关闭:', event.code, event.reason);
};

ws.onerror = function(error) {
    console.error('WebSocket错误:', error);
};
```

### 消息格式

所有WebSocket消息采用JSON格式：

```json
{
  "type": "消息类型",
  "data": "消息数据",
  "timestamp": 1234567890
}
```

### 消息类型

#### 1. 姿态控制消息 (pose6)

**发送格式**:
```json
{
  "type": "pose6",
  "data": {
    "x": 0.05,
    "y": 0.0,
    "z": -0.10,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0
  },
  "timestamp": 1234567890
}
```

**参数说明**:
- `x`, `y`, `z`: 位置坐标，单位：米
- `roll`, `pitch`, `yaw`: 姿态角度，单位：度

**响应格式**:
```json
{
  "type": "pose6",
  "status": "ok",
  "message": "姿态更新成功",
  "timestamp": 1234567891
}
```

#### 2. 动作控制消息 (action)

**发送格式**:
```json
{
  "type": "action",
  "data": {
    "id": 1,
    "beta": 30
  },
  "timestamp": 1234567890
}
```

**参数说明**:
- `id`: 动作ID（1=圆锥动作）
- `beta`: 动作参数（圆锥角度）

**响应格式**:
```json
{
  "type": "action",
  "status": "ok",
  "message": "动作执行成功",
  "timestamp": 1234567891
}
```

#### 3. 预设姿势消息 (preset)

**发送格式**:
```json
{
  "type": "preset",
  "data": {
    "id": 0
  },
  "timestamp": 1234567890
}
```

**参数说明**:
- `id`: 预设ID（0=趴下，1=站立）

**响应格式**:
```json
{
  "type": "preset",
  "status": "ok",
  "message": "姿势切换成功",
  "timestamp": 1234567891
}
```

#### 4. 紧急停止消息 (emergency)

**发送格式**:
```json
{
  "type": "emergency",
  "data": {},
  "timestamp": 1234567890
}
```

**响应格式**:
```json
{
  "type": "emergency",
  "status": "ok",
  "message": "紧急停止执行",
  "timestamp": 1234567891
}
```

#### 5. 心跳消息 (heartbeat)

**发送格式**:
```json
{
  "type": "heartbeat",
  "data": {},
  "timestamp": 1234567890
}
```

**响应格式**:
```json
{
  "type": "heartbeat",
  "status": "ok",
  "timestamp": 1234567891
}
```

#### 6. 状态广播消息 (status)

**服务器主动发送**:
```json
{
  "type": "status",
  "data": {
    "position": {
      "x": 0.05,
      "y": 0.0,
      "z": -0.10
    },
    "attitude": {
      "roll": 0.0,
      "pitch": 0.0,
      "yaw": 0.0
    },
    "battery": 85,
    "temperature": 45,
    "connections": 2
  },
  "timestamp": 1234567890
}
```

### 错误处理

#### 错误响应格式

```json
{
  "type": "error",
  "status": "error",
  "message": "错误描述",
  "code": "ERROR_CODE",
  "timestamp": 1234567891
}
```

#### 常见错误码

| 错误码 | 说明 | 处理建议 |
|--------|------|----------|
| INVALID_MESSAGE | 消息格式错误 | 检查JSON格式 |
| INVALID_PARAMETER | 参数错误 | 检查参数范围 |
| HARDWARE_ERROR | 硬件错误 | 检查硬件连接 |
| SYSTEM_BUSY | 系统忙碌 | 稍后重试 |

### 连接管理

#### 心跳机制

```javascript
// 客户端心跳发送
setInterval(() => {
    if (ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({
            type: 'heartbeat',
            data: {},
            timestamp: Date.now()
        }));
    }
}, 30000); // 30秒间隔
```

#### 自动重连

```javascript
function connectWebSocket() {
    const ws = new WebSocket('ws://192.168.1.7:12580/ws');
    
    ws.onclose = function(event) {
        console.log('连接关闭，3秒后重连...');
        setTimeout(connectWebSocket, 3000);
    };
    
    ws.onerror = function(error) {
        console.error('连接错误:', error);
    };
}
```

---

## 📊 性能指标

### 响应时间

| 接口类型 | 平均响应时间 | 最大响应时间 |
|----------|-------------|-------------|
| HTTP API | 50-100ms | 200ms |
| WebSocket | 10-30ms | 50ms |
| 姿态控制 | 20-40ms | 80ms |
| 动作执行 | 100-200ms | 500ms |

### 并发限制

| 资源 | 限制 | 说明 |
|------|------|------|
| HTTP连接 | 8个 | 同时处理的HTTP请求 |
| WebSocket连接 | 4个 | 同时活跃的WebSocket连接 |
| 控制频率 | 10Hz | 最大控制指令频率 |
| 消息队列 | 100条 | 待处理消息队列长度 |

---

## 🔧 开发示例

### JavaScript客户端示例

```javascript
class RobotController {
    constructor(host = '192.168.1.7', port = 12580) {
        this.host = host;
        this.port = port;
        this.ws = null;
        this.connected = false;
    }
    
    // 连接WebSocket
    connect() {
        const wsUrl = `ws://${this.host}:${this.port}/ws`;
        this.ws = new WebSocket(wsUrl);
        
        this.ws.onopen = () => {
            this.connected = true;
            console.log('机器人连接成功');
        };
        
        this.ws.onmessage = (event) => {
            const message = JSON.parse(event.data);
            this.handleMessage(message);
        };
        
        this.ws.onclose = () => {
            this.connected = false;
            console.log('机器人连接断开');
            setTimeout(() => this.connect(), 3000);
        };
    }
    
    // 发送控制指令
    sendCommand(type, data) {
        if (!this.connected) {
            return this.sendHttpCommand(type, data);
        }
        
        const message = {
            type: type,
            data: data,
            timestamp: Date.now()
        };
        
        this.ws.send(JSON.stringify(message));
    }
    
    // HTTP降级方案
    async sendHttpCommand(type, data) {
        let url = `http://${this.host}:${this.port}`;
        
        switch (type) {
            case 'pose6':
                // 发送多个HTTP请求更新姿态
                await fetch(`${url}/api/pos/x?value=${data.x * 1000}`);
                await fetch(`${url}/api/pos/y?value=${data.y * 1000}`);
                await fetch(`${url}/api/pos/z?value=${(data.z + 0.1) * (-1000)}`);
                await fetch(`${url}/api/rot/roll?value=${data.roll}`);
                await fetch(`${url}/api/rot/pitch?value=${data.pitch}`);
                await fetch(`${url}/api/rot/yaw?value=${data.yaw}`);
                break;
            case 'action':
                await fetch(`${url}/api/action/${data.id}?beta=${data.beta}`);
                break;
            case 'preset':
                await fetch(`${url}/api/preset/${data.id}`);
                break;
            case 'emergency':
                await fetch(`${url}/api/emergency/stop`);
                break;
        }
    }
    
    // 控制机器人姿态
    setPose(x, y, z, roll, pitch, yaw) {
        this.sendCommand('pose6', {
            x: x, y: y, z: z,
            roll: roll, pitch: pitch, yaw: yaw
        });
    }
    
    // 执行动作
    executeAction(actionId, beta = 30) {
        this.sendCommand('action', {
            id: actionId,
            beta: beta
        });
    }
    
    // 切换预设姿势
    setPreset(presetId) {
        this.sendCommand('preset', {
            id: presetId
        });
    }
    
    // 紧急停止
    emergencyStop() {
        this.sendCommand('emergency', {});
    }
    
    // 处理服务器消息
    handleMessage(message) {
        switch (message.type) {
            case 'status':
                console.log('机器人状态:', message.data);
                break;
            case 'error':
                console.error('机器人错误:', message.message);
                break;
            default:
                console.log('收到消息:', message);
        }
    }
}

// 使用示例
const robot = new RobotController();
robot.connect();

// 控制机器人
robot.setPose(0.05, 0, -0.1, 0, 0, 0);  // 设置姿态
robot.executeAction(1, 25);              // 执行圆锥动作
robot.setPreset(1);                      // 站立姿势
robot.emergencyStop();                   // 紧急停止
```

### Python客户端示例

```python
import asyncio
import websockets
import json
import requests
from typing import Optional

class RobotController:
    def __init__(self, host: str = '192.168.1.7', port: int = 12580):
        self.host = host
        self.port = port
        self.ws: Optional[websockets.WebSocketServerProtocol] = None
        self.connected = False
        
    async def connect(self):
        """连接WebSocket"""
        ws_url = f"ws://{self.host}:{self.port}/ws"
        try:
            self.ws = await websockets.connect(ws_url)
            self.connected = True
            print("机器人连接成功")
            
            # 启动消息处理任务
            asyncio.create_task(self.message_handler())
            
        except Exception as e:
            print(f"连接失败: {e}")
            self.connected = False
    
    async def message_handler(self):
        """处理接收到的消息"""
        try:
            async for message in self.ws:
                data = json.loads(message)
                await self.handle_message(data)
        except websockets.exceptions.ConnectionClosed:
            print("WebSocket连接关闭")
            self.connected = False
    
    async def send_command(self, msg_type: str, data: dict):
        """发送控制指令"""
        if not self.connected:
            return self.send_http_command(msg_type, data)
        
        message = {
            'type': msg_type,
            'data': data,
            'timestamp': int(asyncio.get_event_loop().time() * 1000)
        }
        
        await self.ws.send(json.dumps(message))
    
    def send_http_command(self, msg_type: str, data: dict):
        """HTTP降级方案"""
        base_url = f"http://{self.host}:{self.port}"
        
        try:
            if msg_type == 'pose6':
                requests.get(f"{base_url}/api/pos/x", params={'value': data['x'] * 1000})
                requests.get(f"{base_url}/api/pos/y", params={'value': data['y'] * 1000})
                requests.get(f"{base_url}/api/pos/z", params={'value': (data['z'] + 0.1) * (-1000)})
                requests.get(f"{base_url}/api/rot/roll", params={'value': data['roll']})
                requests.get(f"{base_url}/api/rot/pitch", params={'value': data['pitch']})
                requests.get(f"{base_url}/api/rot/yaw", params={'value': data['yaw']})
            elif msg_type == 'action':
                requests.get(f"{base_url}/api/action/{data['id']}", params={'beta': data['beta']})
            elif msg_type == 'preset':
                requests.get(f"{base_url}/api/preset/{data['id']}")
            elif msg_type == 'emergency':
                requests.get(f"{base_url}/api/emergency/stop")
        except Exception as e:
            print(f"HTTP请求失败: {e}")
    
    async def set_pose(self, x: float, y: float, z: float, 
                      roll: float, pitch: float, yaw: float):
        """设置机器人姿态"""
        await self.send_command('pose6', {
            'x': x, 'y': y, 'z': z,
            'roll': roll, 'pitch': pitch, 'yaw': yaw
        })
    
    async def execute_action(self, action_id: int, beta: float = 30):
        """执行动作"""
        await self.send_command('action', {
            'id': action_id,
            'beta': beta
        })
    
    async def set_preset(self, preset_id: int):
        """设置预设姿势"""
        await self.send_command('preset', {
            'id': preset_id
        })
    
    async def emergency_stop(self):
        """紧急停止"""
        await self.send_command('emergency', {})
    
    async def handle_message(self, message: dict):
        """处理服务器消息"""
        msg_type = message.get('type')
        if msg_type == 'status':
            print(f"机器人状态: {message['data']}")
        elif msg_type == 'error':
            print(f"机器人错误: {message['message']}")
        else:
            print(f"收到消息: {message}")

# 使用示例
async def main():
    robot = RobotController()
    await robot.connect()
    
    # 控制机器人
    await robot.set_pose(0.05, 0, -0.1, 0, 0, 0)  # 设置姿态
    await asyncio.sleep(2)
    
    await robot.execute_action(1, 25)              # 执行圆锥动作
    await asyncio.sleep(10)
    
    await robot.set_preset(1)                      # 站立姿势
    await asyncio.sleep(2)
    
    await robot.emergency_stop()                   # 紧急停止

if __name__ == "__main__":
    asyncio.run(main())
```

---

*本API参考文档提供了ESP32四足机器人网页实时控制系统的完整接口说明，包括HTTP RESTful API和WebSocket实时通信协议，以及详细的开发示例。*