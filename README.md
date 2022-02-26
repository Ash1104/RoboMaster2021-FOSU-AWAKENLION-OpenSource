# 佛山科学技术学院醒狮战队2021赛季步兵视觉开源

## 致谢

首先感谢各高校开源代码为本套代码提供的参考，以及感谢2018、2019、2020赛季算法组师兄们的努力。秉承着开源精神，促进各战队间技术交流提升，我队决定本赛季开源本套代码，希望对其他战队提供一定的参考价值。

## 说明

- 本套代码是佛山科学技术学院醒狮战队2021赛季开源代码，仅提供战队间技术交流使用，不得用于任何商业行为。
- 本套代码主要含有：装甲板识别、大小能量机关击打等。
- 介于当前开源情况，本套代码着重介绍装甲板识别和大小能量机关击打逻辑等，其他部分不进行具体说明，如对代码有理解上的问题以及bug等，可联系：
  - 包先宏（Ash-1104）
  - 邢邓鸿（GuaiWoLo92）
- 我们步兵选用的相机分辨率是752 x 480，帧率最高108FPS，镜头是4mm定焦镜头。

## 目录

[1.主要功能介绍及效果展示](#1主要功能介绍及效果展示)

[2.依赖环境](#2.依赖环境)

[3.整体框架](#3.整体框架)

[4.实现方案及原理介绍](#4.实现方案及原理介绍)

[5.代码规范](#5.代码规范)

[6.未来展望](#6.未来展望)

## 1.主要功能介绍及效果展示

- **装甲板识别跟踪：** 主要分为预处理、灯条检测、装甲板检测、择优、扩展、角度解算这六个部分。预处理部分使用的就算正片叠底+迭代法，灯条检测和装甲板检测基本上都是长宽比等常用的筛选调剂。在非DEBUG_MODE下能够跑满相机帧率（108FPS+），所以就没有再继续加ROI了。

![](https://github.com/Ash1104/RoboMaster2021-FOSU-AWAKENLION-OpenSource/blob/master/save_pic/装甲板识别展示.gif)

<center>
图1-1 装甲板识别跟踪效果展示  
</center>  




- **小陀螺跟踪击打：** 前期的预测都集中在卡尔曼滤波上，由于上车测试时间较迟，后面发现卡尔曼滤波的效果不能达到想象中的效果，调不明白，又因为马上要比赛了，所以退而求其次做了一个强行的“扩展装甲板”，简单来说就是将解算的装甲板向前调整一点，效果还是非常不错，但是没有自适应，但是打击一级的麦轮小陀螺效果还算比较理想，能够有较高的命中率。

![](https://github.com/Ash1104/RoboMaster2021-FOSU-AWAKENLION-OpenSource/blob/master/save_pic/反小陀螺功能展示.gif)

<center>
图1-2 小陀螺跟踪击打效果展示  
</center>  




- **能量机关：** 能量机关依然使用的是一些特征筛选，装甲板长宽比、父轮廓角点数目、父轮廓外接矩形长宽比等。分为了初始化状态、移动状态、击打状态和等待状态。在调的好的情况下，极限是一块待激活装甲板2.5s内可以尝试3次击打。大能量机关的拟合时间可以自己调试，目前我们测试能够在10~20帧左右完成拟合。

  ![](https://github.com/Ash1104/RoboMaster2021-FOSU-AWAKENLION-OpenSource/blob/master/save_pic/能量机关击打展示.gif)

  <center>
  图1-3.1 小能量机关击打效果展示  
  </center>  
  
  ![](https://github.com/Ash1104/RoboMaster2021-FOSU-AWAKENLION-OpenSource/blob/master/save_pic/大能量机关击打.gif)
  
  <center>
  图1-3.2 大能量机关击打效果展示  
  </center>  

  


## 2.依赖环境

- **硬件**

| 硬件     | 型号               | 参数                            |
| -------- | ------------------ | ------------------------------- |
| 运算平台 | NUC                | i5                              |
| 工业相机 | 迈德威视MV-UBS31GC | 分辨率752 x 480，最高帧率108fps |
| 镜头     | \                  | 4mm定焦                         |

- **软件**

| 软件类型 | 型号版本         |
| -------- | ---------------- |
| OS       | Ubuntu 16.04     |
| IDE      | Qt Creator-5.8.0 |
| Library  | OpenCV 3.4.1     |

## 3.整体框架

```
│  main.cpp                            // 主函数
├─AngleSolver                          // 角度解算相关
│      bayesEstimateVelocity.cpp       // 贝叶斯估计源文件
│      bayesEstimateVelocity.h         // 贝叶斯估计头文件
│      GravityCompensateResolve.cpp    // 重力补偿源文件
│      GravityCompensateResolve.h      // 重力补偿头文件
│      PnpSolver.cpp                   // PNP解算源文件
│      PnpSolver.h                     // PNP解算头文件
├─ArmorDetector                        // 装甲板检测相关
│      ArmorDetector.cpp               // 装甲板检测源文件
│      ArmorDetector.h                 // 装甲板检测头文件
│      TargetDetection.cpp             // 目标检测源文件
│      TargetDetection.h               // 目标检测头文件
├─calibration                          // 相机标定参数
├─Camera                               // 工业相机API等
│      CameraApi.h                     // API
│      MVVideoCapture.cpp              // 迈德威视工业相机相关
│      MVVideoCapture.h                // 迈德威视工业相机相关
├─config_file                          // 配置参数
├─debug                                // 编译文件夹
├─Gui                                  // 调试界面相关
│      Gui.cpp                         // 调试界面源文件
│      Gui.h                           // 调试界面头文件
├─Preprocessing                        // 预处理相关
│      Preprocessing.cpp               // 预处理相关源文件
│      Preprocessing.h                 // 预处理相关头文件
├─RuneDetector                         // 能量机关检测相关
│      RuneDetector.cpp                // 能量机关检测源文件
│      RuneDetector.h                  // 能量机关检测头文件
├─save_pic                             // 图像保存路径
├─save_video                           // 视频保存路径
├─Serial                               // 通信相关
│      InfantryInfo.cpp                // 通信相关
│      InfantryInfo.h                  // 通信相关
│      JudgementInfo.h                 // 通信相关
│      PackData.cpp                    // 通信相关
│      PackData.h                      // 通信相关
│      Protocol.cpp                    // 通信相关
│      Protocol.h                      // 通信相关
│      Serial.cpp                      // 通信相关
│      Serial.h                        // 通信相关
├─Settings                             // 主要设置相关
│      Settings.cpp                    // 主要设置源文件
│      Settings.h                      // 主要设置头文件
├─template_path                        // 模板匹配相关
```

## 4.实现方案及原理介绍

#### **装甲板识别：**

装甲板识别主要识别机器人最明显的灯条部分，而识别灯条则分为预处理和特征提取两大部 分，首先对图像进行预处理，接着对处理后的图像进行特征提取提取灯条，找到符合条件的 灯条后对灯条间拟合装甲板。

**1、预处理**

预处理部分主要是将图像二值化，而二值化的关键就在于阈值，这里我们采用了正片叠底+ 迭代法，将原图像素 bgr 三通道的权重转化为 0.1、0.2、0.7，接着找出图像最大灰度，最终值：最大灰度 x 0.6+平均灰度 x 0.4，就是二值化的阈值，经过测试，这个方法可以将绝 大多数不发光的物体排除在外，在后面处理轮廓环节能节省较多时间。

**2、特征提取**

特征提取部分主要是对轮廓进行处理，首先一次筛选，使用传统方法过滤大部分明显不是目标的区域，为后续的分类减小干扰和运算量；然后二次筛选找出敌方颜色的灯柱；最后根据灯柱两两匹配找出所有待定装甲板，并选择最优目标进行辅助瞄准。

**1）一次筛选**

- 对所有轮廓长度进行筛选，大于最大阈值或小于最小阈值轮廓排除；
- 对所有轮廓最小包围矩形长宽比进行筛选，大于最大阈值或小于最小阈值轮廓排除；
- 对所有轮廓面积进行筛选，大于最大阈值或小于最小阈值轮廓排除；
- 对轮廓矩形度（轮廓面积与轮廓最小外包矩形面积之比）进行筛选，大于最大阈值或小于最小阈值轮廓排除；
- 对轮廓似圆度（轮廓面积与轮廓长度之比）进行筛选，大于最大阈值或小于最小 阈值轮廓排除；
- 多边形逼近，将轮廓拟合多边形，对多边形顶点数量做筛选，小于阈值轮廓排除。

**2）二次筛选**

- 对轮廓外包矩形进行角度筛选，偏移角过大则排除；
- 对轮廓颜色进行筛选，利用轮廓外包矩形四个点的坐标在原图中计算 b 通道和 r 通道的像素差值，筛选出敌方颜色灯柱。

**3）装甲板检测**

- 对灯条两两匹配；
- 对两灯条高度进行筛选，高度差大于最大阈值排除；
- 对两灯条角度差进行筛选，角度差大于最大阈值排除；
- 将两灯条在 x 轴上的差值与灯条高度之比进行筛选，比值小于最小阈值或大于最大阈值则排除；
- 将两灯条在 y 周上的差值与 x 轴上的差值之比进行筛选，比值大于最大阈值则排除；
- 拟合包含两灯条在内的最小矩形，对矩形面积进行筛选，小于最小阈值则排除；
- 利用两灯条计算出中间装甲板 4 个顶点位置；
- 对装甲板矩形宽高比进行筛选；
- 找到装甲板。

**装甲板识别逻辑流程图**

![](https://github.com/Ash1104/RoboMaster2021-FOSU-AWAKENLION-OpenSource/blob/master/save_pic/装甲板识别逻辑流程图.png)

<center>
图4-1 装甲板识别逻辑流程图  
</center>  




**目前仍存在的问题**

- 灯条变形问题，这是这次对抗赛录制的一段视频，可以看到灯条已经成椭圆状了，已经背离了我们的识别逻辑。由于灯条发光，降低相机的曝光时间能够一定程度上缓解这种现象，但是这样也不是特别理想。

![](https://github.com/Ash1104/RoboMaster2021-FOSU-AWAKENLION-OpenSource/blob/master/save_pic/灯条变形.gif)

<center>
图4-2 灯条变形问题  
</center>  




- 场地灯光误识别问题，这是一直以来都有的问题，是很难避免的，下图为某一场地灯光。其实也能隐隐约约的感觉到，组委会正在努力将大家从传统的OpenCV推向神经网络方向。

![](https://github.com/Ash1104/RoboMaster2021-FOSU-AWAKENLION-OpenSource/blob/master/save_pic/场地灯光误识别.png)

<center>
图4-3 场地灯光误识别问题  
</center>  




#### 能量机关：

（大能量机关推导中，运用了公式，由于没有插件GitHub无法自动渲染mathJax”，所以这一部分建议下载“[大能量机关推导.pdf](大能量机关推导.pdf)”查阅，或打开“[大能量机关推导.png](save_pic/大能量机关推导.png)”查看，或者下载到本地使用软件进行渲染）
- 目前已改成图片形式，建议在白色背景下阅览

**能量机关先导问题：**

其实能量机关的主要问题就算在T秒内，能量机关转过的角度。（T：发弹延时+弹丸滞空时间+能量机关亮起时间+云台移动时间等）

小能量机关转速固定为![](https://www.zhihu.com/equation?tex=%5C%2010RPM%20)，T秒内转动的角度即为![](https://www.zhihu.com/equation?tex=%5C%2060%C2%B0%20%2A%20T%20)。

大能量机关转速按照三角函数呈周期变化。速度目标函数为：![](https://www.zhihu.com/equation?tex=%5C%20spd%20%3D%200.785%20%E2%88%97%20sin%20%281.884%20%E2%88%97%20t%29%20%2B%201.305)，其中 ![](https://www.zhihu.com/equation?tex=%5C%20spd%20)的单位为![](https://www.zhihu.com/equation?tex=%5C%20rad/s%20)，![](https://www.zhihu.com/equation?tex=%5C%20t%20)的单位为![](https://www.zhihu.com/equation?tex=%5C%20s%20)。

假设速度目标函数为：![](https://www.zhihu.com/equation?tex=%5C%20spd%20%3D%20Asin%28%5Comega%20x%20%2B%20%5Cvarphi%29%20%2B%20C)，其中 ![](https://www.zhihu.com/equation?tex=%5C%20spd%20)的单位为![](https://www.zhihu.com/equation?tex=%5C%20rad/s%20)，![](https://www.zhihu.com/equation?tex=%5C%20t%20)的单位为![](https://www.zhihu.com/equation?tex=%5C%20s%20)。

进行如下公式推导：

假设初始时间为![](https://www.zhihu.com/equation?tex=%5C%20t_1%20)，末尾时间为![](https://www.zhihu.com/equation?tex=%5C%20t_2%20)，对速度目标函数![](https://www.zhihu.com/equation?tex=%5C%20spd%20%3D%20Asin%28%5Comega%20x%20%2B%20%5Cvarphi%29%20%2B%20C)进行积分，得![](https://www.zhihu.com/equation?tex=%5C%20t_1%20)到![](https://www.zhihu.com/equation?tex=%5C%20t_2%20)能量机关总共转过的角度差![](https://www.zhihu.com/equation?tex=%5C%20%5CDelta%20%5Ctheta)为：

![](https://www.zhihu.com/equation?tex=%0A%5Cbegin%7Baligned%7D%0A%5CDelta%20%5Ctheta%3D%5Cint_%7Bt_%7B1%7D%7D%5E%7Bt_%7B2%7D%7D%20s%20p%20d%20%26%3D-%5Cfrac%7BA%7D%7B%5Comega%7D%5Cleft%5B%5Ccos%20%5Cleft%28%5Comega%20t_%7B2%7D%2B%5Cvarphi%5Cright%29-%5Ccos%20%5Cleft%28%5Comega%20t_%7B1%7D%2B%5Cvarphi%5Cright%29%5Cright%5D%2BC%5Cleft%28t_%7B2%7D-t_%7B1%7D%5Cright%29%20%5C%5C%0A%26%3D%5Cfrac%7B2%20A%7D%7B%5Comega%7D%5Cleft%5B%5Csin%20%5Cfrac%7B%5Comega%5Cleft%28t_%7B1%7D%2Bt_%7B2%7D%5Cright%29%2B2%20%5Cvarphi%7D%7B2%7D%20%5Csin%20%5Cfrac%7B%5Comega%5Cleft%28t_%7B2%7D-t_%7B1%7D%5Cright%29%7D%7B2%7D%5Cright%5D%2BC%5Cleft%28t_%7B2%7D-t_%7B1%7D%5Cright%29%0A%5Cend%7Baligned%7D%0A)

设![](https://www.zhihu.com/equation?tex=%5C%20%5CDelta%20t%20%3D%20t_2%20-%20t_1%20)，得：

![](https://www.zhihu.com/equation?tex=%0A%5CDelta%20%5Ctheta%3D%5Cfrac%7B2%20A%7D%7B%5Comega%7D%5Cleft%5B%5Csin%20%5Cfrac%7B%5Comega%5Cleft%28%5CDelta%20t%2B2%20t_%7B1%7D%5Cright%29%2B2%20%5Cvarphi%7D%7B2%7D%20%5Csin%20%5Cfrac%7B%5Comega%20%5CDelta%20t%7D%7B2%7D%5Cright%5D%2BC%20%5CDelta%20t%0A)

即：

![](https://www.zhihu.com/equation?tex=%0A%5Cfrac%7B%5Comega%7D%7B2%20A%7D%28%5CDelta%20%5Ctheta-C%20%5CDelta%20t%29%3D%5Csin%20%5Cleft%5B%5Cfrac%7B%5Comega%7D%7B2%7D%5Cleft%28%5CDelta%20t%2B2%20t_%7B1%7D%5Cright%29%2B%5Cvarphi%5Cright%5D%20%5Csin%20%5Cfrac%7B%5Comega%20%5CDelta%20t%7D%7B2%7D%20%5C%5C%0A)

即：

![](https://www.zhihu.com/equation?tex=%0A%5Carcsin%20%5Cleft%5B%5Cfrac%7B%5Comega%28%5CDelta%20%5Ctheta-C%20%5CDelta%20t%29%7D%7B2%20A%20%5Csin%20%5Cfrac%7B%5Comega%20%5CDelta%20t%7D%7B2%7D%7D%5Cright%5D%3D%5Cfrac%7B%5Comega%7D%7B2%7D%5Cleft%28%5CDelta%20t%2B2%20t_%7B1%7D%5Cright%29%2B%5Cvarphi%0A)

化简出![](https://www.zhihu.com/equation?tex=%5C%20t_1%20)，得：

![](https://www.zhihu.com/equation?tex=%0At_%7B1%7D%3D%5Cfrac%7B1%7D%7B%5Comega%7D%5Cleft%5B%5Carcsin%20%5Cfrac%7B%5Comega%28%5CDelta%20%5Ctheta-C%20%5CDelta%20t%29%7D%7B2%20A%20%5Csin%20%5Cfrac%7B%5Comega%20%5CDelta%20t%7D%7B2%7D%7D%5Cright%5D-%5Cvarphi-%5Cfrac%7B%5CDelta%20t%7D%7B2%7D%0A)

由于官方给定的速度目标函数为：![](https://www.zhihu.com/equation?tex=%5C%20spd%20%3D%200.785%20%E2%88%97%20sin%20%281.884%20%E2%88%97%20t%29%20%2B%201.305)，代入推导公式，得：

![](https://www.zhihu.com/equation?tex=%0At_%7B1%7D%3D%5Cfrac%7B1%7D%7B1.884%7D%5Cleft%5B%5Carcsin%20%5Cfrac%7B1.884%20%5Ctimes%28%5CDelta%20%5Ctheta-1.305%20%5Ctimes%20%5CDelta%20t%29%7D%7B1.57%20%5Csin%20%280.942%20%5CDelta%20t%29%7D%5Cright%5D-%5Cfrac%7B%5CDelta%20t%7D%7B2%7D%0A)



![](https://www.zhihu.com/equation?tex=%0A%5CDelta%20%5Ctheta%3D%5Cfrac%7B1.57%7D%7B1.884%7D%5Cleft%5B%5Csin%20%5Cleft%280.942%20%5CDelta%20t%2B1.884%20t_%7B1%7D%5Cright%29%20%5Csin%20%280.942%20%5CDelta%20t%29%5Cright%5D%2B1.305%20%5CDelta%20t%0A)


因此，仅需知道角度差![](https://www.zhihu.com/equation?tex=%5C%20%5CDelta%5Ctheta%20)和时间差![](https://www.zhihu.com/equation?tex=%5C%20%5CDelta%20t%20)，就可以得到![](https://www.zhihu.com/equation?tex=%5C%20t_1%20)，即得到![](https://www.zhihu.com/equation?tex=%5C%20t_1%20)在能量机关转动周期中真正对应的时间，记为![](https://www.zhihu.com/equation?tex=%5C%20t_1)(周期)，而非程序计时得到的![](https://www.zhihu.com/equation?tex=%5C%20t_1%20)，记为![](https://www.zhihu.com/equation?tex=%5C%20t_1%20)(程序计时器)。![](https://www.zhihu.com/equation?tex=%5C%20t_1%20)(周期)固定不变，在某一时刻，此时计时为![](https://www.zhihu.com/equation?tex=%5C%20t_2)(程序计时器)，再加上初始程序计时的![](https://www.zhihu.com/equation?tex=%5C%20t_1%20)(程序计时器)，得到两者的时间差![](https://www.zhihu.com/equation?tex=%5C%20%5CDelta%20t%20)，通过![](https://www.zhihu.com/equation?tex=%5C%20%5CDelta%20t%20%2B%20t_1)(周期)，就得到当前时刻对应于周期中的时间，记为![](https://www.zhihu.com/equation?tex=%5C%20t_2%20)(周期)。通过![](https://www.zhihu.com/equation?tex=%5C%20t_2)(周期)代表![](https://www.zhihu.com/equation?tex=%5C%20t_1%20)(周期)，时间间隔为![](https://www.zhihu.com/equation?tex=%5C%20T%20)代表![](https://www.zhihu.com/equation?tex=%5C%20%5CDelta%20t%20)，代入公式：

![](https://www.zhihu.com/equation?tex=%0A%5CDelta%20%5Ctheta%3D%5Cfrac%7B1.57%7D%7B1.884%7D%5Cleft%5B%5Csin%20%5Cleft%280.942%20%5CDelta%20t%2B1.884%20t_%7B1%7D%5Cright%29%20%5Csin%20%280.942%20%5CDelta%20t%29%5Cright%5D%2B1.305%20%5CDelta%20t%0A)

即得到当前时刻起，经过时间间隔T后，大能量机关转动过的角度。



**能量机关识别思路：**

- 1 输入图片；

- 2 预处理：
  - 2.1 根据目标颜色，进行红蓝通道对应相减得到对应的灰度图，如能量机关为红色， 则用红通道减蓝通道，蓝色则相反；
  - 2.2 具体颜色可以从电控通信兵种信息数据获取到敌方颜色信息，比如敌方颜色为红 色，则能量机关颜色为蓝色；
  - 2.3 对灰度图进行阈值分割，得到最终的二值图。

- 3 特征提取（检查装甲板）：
  - 通过识别到的轮廓集，遍历轮廓集，拟合出外包矩形，进行轮廓长度、矩形长宽比、面 积比（轮廓面积 / 外包矩形面积）、轮廓面积等条件的阈值比较，筛选出符合条件（与装甲板相似）的轮廓；
- 4 找到装甲板轮廓：
  - 4.1 找到 1 个装甲板轮廓，判断有无父轮廓：
    - 有，则判断父轮廓面积是否符合条件，符合再对父轮廓拟合多边 形，计算其角度数，判断角点数是否符合条件（7~10 左右），符合则判断父轮廓 的面积比是否符合条件，符合则存储装甲板（非父轮廓）的旋转矩形；
    - 无，则下一次识别。
  - 4.2 找到多个装甲板轮廓，遍历装甲板轮廓集：
    - 后续与4.1相同。
  - 4.3 其余时候，皆为识别失败。

**能量机关识别逻辑流程图如下图所示**

![](https://github.com/Ash1104/RoboMaster2021-FOSU-AWAKENLION-OpenSource/blob/master/save_pic/能量机关识别逻辑图.png)

<center>
图4-4 能量机关识别逻辑流程图  
</center>  






**能量机关击打逻辑流程图：**

![](https://github.com/Ash1104/RoboMaster2021-FOSU-AWAKENLION-OpenSource/blob/master/save_pic/能量机关初始化逻辑流程图.png)

<center>
图4-5 能量机关初始化逻辑流程图  
</center>  




![](https://github.com/Ash1104/RoboMaster2021-FOSU-AWAKENLION-OpenSource/blob/master/save_pic/能量机关状态切换击打逻辑流程图.png)

<center>
图4-6 能量机关状态切换逻辑流程图  
</center>  


## 5.代码规范

**命名风格**

|            | 风格        | 例子                                                         |
| ---------- | ----------- | ------------------------------------------------------------ |
| **文件名** | 大驼峰      | AngleSolver.hpp                                              |
| **类名**   | 大驼峰      | class AngleSolver{}                                          |
| **方法**   | 小驼峰      | void setTargetSize(double width, double height);<br/>void setCameraParam(const cv::Mat & camera_matrix, const cv::Mat &dist_coeff); |
| **变量**   | 小写+下划线 | cv::Mat cam_matrix;<br/>cv::Mat distortion_coeff;<br/>double width_target;<br/>double height_target; |
| **宏**     | 大写+下划线 | \#define TRUNC_ABS(a) ((a) > 0 ? (a) : 0);                   |

**方法常用前缀**

| 方法        | 说明                                                         |
| ----------- | ------------------------------------------------------------ |
| initXX()    | 初始化相关方法,使用init为前缀标识，如初始化布局initView()    |
| isXX()      | checkXX()方法返回值为boolean型的请使用is或check为前缀标识    |
| getXX()     | 返回某个值的方法，使用get为前缀标识                          |
| processXX() | 对数据进行处理的方法，使用process为前缀标识，或使用Proc为后缀标识 |
| saveXX()    | 与保存数据相关的，使用save为前缀标识                         |
| resetXX()   | 对数据重组的，使用reset前缀标识                              |
| clearXX()   | 清除数据相关的                                               |
| removeXXX() | 清除数据相关的                                               |
| writeXX()   | 写入文件相关                                                 |
| readXX()    | 读入文件相关                                                 |

**注释规范**

| 场景     | 规范                                                         |
| -------- | ------------------------------------------------------------ |
| 代码块   | 统一规范：<br />//-----------------------------------【标题】----------------<br/>// 描述：<br/>//----------------------------------------------------------------- |
| 方法     | 统一规范<br/> /**<br/> \* @brief 这里书写该方法的简介，包括其作用等<br/> \* @param 第一个参数说明<br/> * @param第二个参数说明<br/> \* @return 返回值说明<br/> \* @author 参与开发人员<br/> \* @date 2021.8.4<br/> */ |
| 宏定义   | 例： #define SHOW_IMAGE //如果注释掉则不显示图片，有利于加快运行速度 |
| 成员变量 | 如果变量名不能很好的显示其含义或有特殊用途，就需要注释       |
| 其他     | 不容易理解的逻辑部分                                         |

**常用代码书写规范**

- ”,“之后要留空格，for语句中”;“后需要留空格；
- 赋值操作符、比较操作符、算数操作符、逻辑操作符等二元操作符的前后应留空格；
- 一元操作符前后不加空格；
- 大括号换行，如果if、for等后只跟有一行的可不加大括号
- 其他书写规范尽量与Visual Studio中默认书写规范一致。

## 6.未来展望

- 继续研究反小陀螺算法，提高反小陀螺算法的自适应程度，自动化程度，提高反小陀螺算法击打命中率；
- 将整体算法从传统算法向神经网络转移；
- 对移动目标，继续研究或发现新的解决方案，提高击打移动目标的命中率；
- 在装甲板识别中加入数字识别，降低误识别概率。
