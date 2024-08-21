# Dynamic ROSTopic Subscriber
Dynamic ROSTopic Subscriber 是一个 Python 库，用于简化 ROS（机器人操作系统）主题的动态订阅过程。它允许用户轻松订阅任何 ROS 主题，而无需预先知道消息类型。  

## 特性

- 动态订阅 ROS 主题 
- 自动处理未知的消息类型 
- 使用装饰器简化订阅过程 
- 单例模式确保只有一个订阅器实例 

## 安装 

使用 pip 安装 Dynamic ROS Topic Subscriber： 

```bash 
pip install dynamic_rostopic_subscriber 
``` 

## 使用方法 

以下是一个基本的使用示例： 

```python 
from dynamic_rostopic_subscriber import rostopic_subscriber 

@rostopic_subscriber.subscribe("/some/ros/topic") 
def callback_function(msg): 
    print(f"Received message: {msg}") 
    
if __name__ == "__main__": 
    rostopic_subscriber.spin() 
``` 

## API 参考 

### `@rostopic_subscriber.subscribe(topic, msg_type=AnyMsg)` 

装饰器函数，用于订阅 ROS 主题。 

- `topic`: 要订阅的 ROS 主题名称 
- `msg_type`: 消息类型（可选，默认为 AnyMsg） 

### `rostopic_subscriber.spin()` 

开始处理 ROS 消息。这个方法会阻塞，直到 ROS 节点被关闭。 

## 注意事项 

- 确保在使用此库之前已经正确设置了 ROS 环境。 
- 此库依赖于 `rospy` 和 `genpy`，请确保它们已经安装。 

## 贡献 

欢迎提交问题和拉取请求。对于重大更改，请先开一个 issue 讨论您想要更改的内容。 

## 许可证 

本项目采用 MIT 许可证 - 有关详细信息，请参阅 [LICENSE](LICENSE) 文件。