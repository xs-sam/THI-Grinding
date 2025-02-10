# 使用说明

## 简介:

​	该包是用于管理rosnode的，他可以用于启动和停止rosnode或者是一个roslaunch文件。

## 实现方式:

​	该包的结构实现，是利用ros的服务功能来做的，当该节点启用后，会开启一个名为/nodeManager的服务器，利用ros服务通讯便可以给该节点转入参数来管理其他的节点。

## 服务文件结构:

```xml
string nodeType # launch / node
string packName # 包名
string nodeName # .launch名字 / 节点名字
string nodeState # start / stop
---
bool ok
```



## 使用:

 创建工作空间(已有的话就不用了)

```bash
mkdir -p my_ws/src
```

把该功能包放到你的工作空间的src下

编译

```bash
catkin_make
```

​	1、打开终端

​	2、cd到工作空间下

​	3、刷新环境

```bash
source devel/setup.bash
```

​	4、启动该包

```bash
roslaunch node_manager node_manager.launch
```

​	5、呼叫服务

​		这个时候就可以看到一个名为/nodeManager的服务存在了。

​		只需要呼叫该服务就可以实现了。		
​	如：

1、启动小乌龟节点 等价于 rosrun turtlesim turtlesim_node

```bash
rosservice call /nodeManager "nodeType: 'node'
packName: 'turtlesim'
nodeName: 'turtlesim_node'
nodeState: 'start'" 
```

停止刚刚启动的节点

```bash
rosservice call /nodeManager "nodeType: 'node'
packName: 'turtlesim'
nodeName: 'turtlesim_node'
nodeState: 'stop'" 
```



2、启动launch文件 这个是我自己的一个测试launch 相当于 roslaunch test test.launch

​		nodeName需要去除.launch

```bash
rosservice call /nodeManager "nodeType: 'launch'
packName: 'test'
nodeName: 'test'
nodeState: 'start'" 
```

停止刚刚的launch节点

```bash
rosservice call /nodeManager "nodeType: 'launch'
packName: 'test'
nodeName: 'test'
nodeState: 'stop'" 
```

