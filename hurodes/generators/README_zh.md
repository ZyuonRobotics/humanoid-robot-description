## Generator类

所有的Generator类都应该是基类`GeneratorBase`的子类。

一个Generator类在其生命周期之内主要有三个行为：
- 加载（`load()`）：将外部数据（通常保存在HURODES_ASSETS_PATH路径下）加载进Generator类，由各个子类完成加载和校验逻辑
- 生成（`generate()`）：目前Generator类只支持生成xml文件（mjcf和urdf都是xml文件），`generate`函数用于根据已经加载的信息在`xml_root`上生成元素
- 销毁（`destroy()`）：重置`xml_root`，用于清除生成阶段产生的结果

在Generator基类中，我们实现了`export()`函数，他要求实例已经完成了加载，可以一步到位生成最终的xml字符串并保存到指定路径。

此外，尽管HRDF和Generator都是hurodes的重要组成部分，但是两者的逻辑并没有完全绑定，这是因为其他库（如humanoid-retargeting）也会使用Generator，而其并不依赖HRDF。我们通过HRDFMixin以代码补丁的形式将Generator和HRDF结合。

### 加载

基类`GeneratorBase`实现了`load(**kwargs)`函数，通常情况下子类只需要实现`_load(**kwargs)`，其中传入的参数`**kwargs`完全由`_load`处理。

Generator类有一个名为`_loaded`的属性，其通过`@propert`函数`loaded`被访问，用于表示当前的实例是否已经加载了数据，当外部调用`load()`函数时，基类会自动将`_loaded`设置为`True`。调用`generate`和`export`函数时，其都会断言`loaded`的值为`True`。

在某些特殊情况下可能需要重写`loaded`的逻辑，例如在`MJCFGeneratorComposite`中`loaded`会检测所有持有的Generator是否已经被加载，如果都被加载了则返回`True`。

在实际使用中，可以先构造Generator类，再调用`load()`函数加载数据，例如:
```
generator = MJCFHumanoidGenerator()
generator.load(hrdf)
```

也可以实现`from_*`类方法，在构造的时候就完成加载:
```
generator = MJCFHumanoidGenerator.from_hrdf(hrdf)
```

### 生成

同样的，基类`GeneratorBase`实现了`generate(**kwargs)`函数，通常情况下子类只需要实现`_generate(**kwargs)`，其中传入的参数`**kwargs`完全由`_generate`处理。

通常情况下Generator类只需要将调用`load()`所加载的数据添加到`xml_root`即可。一些特殊情况，如`MJCFGeneratorComposite`，则会依次调用所有持有的Generator。

### 销毁

销毁生成阶段产生的所有结果，但不会影响load阶段加载的数据。理论上，应该满足销毁后再次生成的结果与之前一致:

```
generator.load()
str1 = generator.export(**kwargs)
generator.destroy()
str2 = generator.export(**kwargs) # same args
assert str1 == str2
```

## MJCFGenerator

所有MJCFGenerator都有以下特性：
- simulator_config：`MJCFGeneratorBase`类及其子类都会持有`simulator_config`，用于设定与仿真引擎相关的信息。在`MJCFHumanoidGenerator`类中，`simulator_config`会被从HRDF中读取
- 不使用defualt：为了简化MJCFGenerator及其子类的逻辑，我们不在MJCF中使用`<defualt>`，这一定程度上降低了其生成的MJCF的可读性，但是我们同样不鼓励在正常情况下维护和分发MJCF（你只应该维护和分发HRDF，MJCF则在被使用的时候自动导出）

### prefix和scene

prefix和scene是MJCFGenerator相对于其他Generator类的主要差别。他们都可以在调用`generate()`函数的时候被指定是否启用，例如`MJCFHumanoidGenerator`类的`_generate`函数签名为`_generate(self, prefix=None, add_scene=True)`。

其中prefix可以用来设定生成的`xml`文件的每一个名称(`name`，如`body_name`，`joint_name`，`mesh_name`等等)的前缀。其作用是防止当多个generator通过`MJCFGeneratorComposite`合并时由于名字重复而报错。这在retargeting以及多机器人交互的场景中非常实用。

scene则与mujoco的特性有关，`MJCFGeneratorBase`实现了`add_scene()`函数，用于根据传入的数据（通常是`simulator_config`中的数据）以及默认值生成仿真器中的场景信息，子类可以在`_generate()`中选择是否调用`add_scene()`函数以生成场景。

### MJCFGeneratorComposite

`MJCFGeneratorComposite`用于将多个MJCFGenerator生成的结果合并，从而在同一个仿真引擎中实现多个机器人的交互。其主要有以下特性：
- 不实现加载函数：`MJCFGeneratorComposite`要求传入的generator在外部完成加载，可以是在传入时已经加载好，也可以在传入后在外部加载。其`load`函数只会检测所有持有的generator是否已经加载，不会执行加载逻辑
- 自动合并所有顶级Element：顶级Element指的是在`<mujoco>`标签直接拥有的Element，常见的有`<compiler>`，`<asset>`，`<worldbody>`，`<actuator>`和`<visual>`等
- 合并mesh地址：自动寻找所有generator的mesh文件地址，计算所有mesh地址的公共路径，并修改每个mesh的相对路径
