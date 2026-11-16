### Release notes
SDK版本: tc_platform_sdk Vx.x.x。
此版本SDK支持B80(A1)、B85、B87芯片
这里需要标明，这个版本支持的芯片型号以及对应芯片型号的版本号
版本号的原则如下：我们只写这个软件版本上测试过的芯片版本。
比如B80再V1.3.0版本的时候，只有A0芯片，只测试了A0，所以在1.3.0的release note中我们就只写支持B80（A0）.
在V1.4.0版本的时候，我们只测试了A1版本的芯片，那再V1.4.0版本的release note上，我们只写支持B80（A1）.
但是其实根据芯片的改动说明，A0、A1是不需要软件针对芯片版本做特殊处理的，这个可能针对某些需求的客户，FAE会问，那我们把他放到release note中的note中。
Note这样写：
B80在V1.3.0版本的时候，只有A0芯片，只测试了A0，在V1.4.0版本的时候，我们只测试了A1版本的芯片。
根据芯片的改版说明，A0、A1是不需要软件针对芯片版本做特殊处理的，其实V1.3.0和V1.4.0都是可以用于A0和A1芯片的，
但是因为内部并没有在每个软件版本上做所有芯片版本的验证，所以建议按照release note中标注的芯片版本进行使用。

### Bug Fixes

* **pm** 
  * (B85)解决了睡眠时间小于950us时，不能唤醒的问题。
* **rf** 
  * (B85)修复了函数rf_set_rffe_pin功能异常。

### Features

* **i2c**
  * (B85)增加了master在发ID阶段检测到NACK后发stop，并停止读写动作
  * (B85)在dma demo中增加了tx_done中断

### Refactoring

* **spi**
  * (B85)调整了SPI_Demo中LED的初始化过程。

### BREAKING CHANGES

* **flash**
  * (B85)为了兼容其他sdk，flash_read_mid函数以前是传入一个buf指针，改为返回unsigned int型变量。

### Refactoring

* **spi** 
  * (B85)调整了SPI_Demo中LED的初始化过程。

----

### Bug Fixes

* **pm**
  * (B85)Solved the problem of not being able to wake up when the sleep time is less than 950us.
* **rf**
  * (B85)Fixed an abnormal function of rf_set_rffe_pin function

### Features

* **i2c**
  * (B85)Add function that the master sends a stop after detecting a NACK in the ID phase，then stops reading and writing
  * (B85)Add tx_done interrupt in dma demo

### Refactoring

* **spi**
  * (B85)adjusted the initial process of LED in SPI_Demo.

### BREAKING CHANGES

* **flash**
  * (B85)In order to be compatible with other SDKs, the flash_read_mid function used to pass in a buf pointer, and instead returns an         unsigned int variable.

### Refactoring

* **spi**
  * (B85)adjusted the initialization process of LED in SPI_Demo

----

### 内部说明

### Features

* **flash**
  + (B85)功能说明：
    增加四线读接口，增加两线读接口，增加四线写接口
    flash添加了读供应商函数flash_get_vdd_f_calib_value()，和供应商枚举类型flash_vendor_e
  + (B85)自测项说明（测试条件，测试方法）：
    测试新增加的接口，功能正常

### Bug fixes

* **rf**
  + (B85)问题详细描述：状态寄存器包含加锁的bit和四线配置bit和其他的bit。调用flash的加锁和解锁接口时，原来的接口直接写入加锁和解锁的值，别的状态就会清零
  + (B85)修改方案：先读出状态值，再对加锁的bit配置，其他的bit保持不变，再写入这个状态值。
  + (B85)自测项说明（测试条件，测试方法）：新的程序测试解锁和解锁功能，其他的bit的配置没变。
  + (B85)芯片bug：XXXX 芯片 XXXX SDK 有相同的 bug
  + (B85)软件bug：XXXX 芯片 XXXX SDK 有相同的 bug

### BREAKING CHANGES

* **rf**
  + (B85)接口修改：
  + (B85)结构修改：

### Performance Improvements
* **rf**
* (B85)说明1：
* (B85)说明2：
---
针对要求提交人和审核人的要求，并且审核人需要填写下面的表格
要求序号 | 是否检查(Y-已检查，N-未检查)

| 检查项目| 要求 | 状态 |
| ------ | ------ | ------ |
| Title | 明确commit类型，一次merge request必须是一种类型的修改，不可以混合的，必须有修改的芯片型号，以便后面查找 | N |
| Description|  1. release notes的写法主要是针对使用函数接口的人去看的，所以需要从描述bug和功能的角度出发：<br>（1）例如：修复UART suspend起来或reset UART后，配置函数uart_receive_dma不能继续接收数据问题<br>（2）关于寄存器的操作等说明不需要出现在release note中，因为使用者只关心接口层面的问题 <br> 2. 写出中英文，先写中文再写英文 <br> 3. 对外 Release notes的内容同步到tleink_b91m_driver_sdk_Relase_Note.md 文件里和代码一并上传<br>4、相同类型合并:<br>（1）同类型log,例如 Bug Fix放在一起<br>（2）同一个模块合并在一起,例如PM<br>5、必须明确是针对那个芯片的修改：(B91)、(B92)、(B91 /B92)<br>6、描述是否足以让你清楚问题以及解决方案是什么，描述上是否有什么异议<br>7、对内“ 内部说明” 要求：这里可以用中文，需要考虑清楚易懂 | N | 
| 修改内容 | 修改内容的正确性 | N | 
| 编码规范 | 代码是否符合编程规范，(请先详细阅读编程规范文档），以下列出经常看到的几个问题：<br>1、代码注释需要有问题描述，解决方案，确认人-芯片组同事，修改人，修改日期，寄存器bit定义<br>2、函数头的注释要详细，参数有范围的要写清楚范围，有单位的要写单位，格式要正确并清楚，可以参考flash的注释，flash的注释写的比较清楚。<br>3、寄存器定义以及其bit定义，不仅要在reg.h中定义，并且代码实现的时候，需要用起来，这样可读性会强，可以减少阅读人的时间<br>4、函数接口全不全（根据寄存器定义可判断）特别是新添加的驱动，要结合寄存器的所有功能，检查下驱动接口是否是全面的，都考虑到了。<br>5、函数接口不要有大写字母，变量枚举类型定义等，注意前缀后缀要求。<br>6、将上层功能和应用层供区分开，不要有冗余代码，让代码尽量简单易读，可以共用的地方单独封装函数接口。<br>7、demo写法是否合适<br>8、代码的可读性，保证任何人来看的时候，都比较容易理解，这样方便后面维护，对于就是比较难理解的，添加注释说明辅助。<br>9、写注释的时候注意，驱动注释分为两种，一种是给使用驱动中的人看的，他们只需要关注如何使用，还有一种是给驱动组内人员看的，因为有可能同个模块会多个人负责，为了让组内的人知道修改原因。所以写注释的时候注意位置，一般给使用者看的 写在函数头，给咱们自己看的，写在函数内。| N | 
| 邮件 | 所有的bug修改发邮件给群组，重大的结构调整 或者是什么想要通知sdk的 也可以通过群组通知 | 是否需要邮件通知 |


* **commit类型和release note关键字的关系**

| commit类型  |  release note关键字 | 说明 |
| ------ | ------ | ------ |
| fix| Bug Fixes | 原有代码有bug |
| feat| Features | 新增接口、新增功能 |
| breaking changes| BREAKING CHANGES | 破坏性的变化，会导致向下兼容问题的变化 | 
| refactor| Refactoring | 代码有调整，不影响上层使用，对软件内部结构的一种调整,  目的是在不改变软件可观察行为的前提下, 提高其可靠性, 降低其修改成本 |  
| perf| Performance Improvements | 性能优化 | 
|  | Known issues | 很重大的需要各个SDK更新的严重bug | 
  
* 特别的，既属于**fix**又属于**breaking change**这种merge request，写成**fix+breaking changes**
---



---



  
  
  


