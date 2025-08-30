## Generator类

一个Generator类具有以下行为：
- 加载：通过load函数从指定的路径加载数据
- 生成：根据已有信息在xml-root上生成元素，构建机器人描述文件（mjcf或者urdf）
- 销毁：重置xml-root，用于清除generate的结果

