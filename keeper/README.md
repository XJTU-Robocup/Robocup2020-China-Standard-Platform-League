[TOC]


# 守门员代码解读----GK

## 1、 初始状态

首先解释 initial_state 之后的代码

afterstart1 和 afterstart2 为统一模块，功能是使机器人在刚**开局**后能够迅速找到正确的位置。

（注：根据观察，**倒下重新开始**或是**罚下场重新上场**后不会经历readystate,会从playingstate直接开始，因此仍然会经过这一调整位置的代码，防止混乱）

## 2、整体逻辑

最根本的思想是分区决策，主要执行状态有三类：

* 找到球之前
* 找到球之后
* 刚刚丢球时

除此之外还有一部分意外状态的判断独立于这三个流程模块之外。

接下来会一部分一部分进行解释。

----

### 2.1找到球之前

在找到球之前，主要需要这几个功能:

* 快速找球（**search**）/全面找球(**ownGoal1**)，考虑效率问题交替进行
* 归位(**walkToSearch**)，即回到合适位置找球

#### search 快速找球

正如它的名字，就是朴素的找球，只动脑袋不动身子，优点速度极快，缺点范围小适合需要短时间空快速搜索的场景

#### ownGoal1 全面找球

名字是乱起的我也不记得为什么了，改起来太烦懒得动了

#### walkToSearch

所见即所得，功能就是走去找球，简单粗暴回位置（theFieldDimensions.xPosOwnPenaltyArea,0），也就是禁区线中点上

----

### 2.2找到球之后

找到球之后，根据球的位置选择状态

* **safenow **球很远时安全！实际上看不到这么远基本不存在这种情况，但是以防万一

* **readystate** 球相对远时做好准备，这个时候进球概率一般不大，但是仍有可能

* **warningstate** 球相对近了，开始变危险！

* **kickball** 在踢球范围里，可以踢了！

* **keepGuardLeft/Right** 球的位置较近但很偏，踢容易空门，站在合适位置可以很容易挡住大部分进球空间。

  ​											（说明：**keepGuard2，keepGuard3** 是后续动作帮助走到位，不单独说明）

  (再说明：还有一个重要的东西是 **walkToSearch** 在这里的用处是近处丢球时防止乌龙球)

#### safenow

没啥功能，毕竟很安全没烦恼，唯一功能是在安全时晃悠回中心保证视野

#### readystate

功能也不复杂，球相对远比较安全，将球的位置映射到防守线上，选点防守。

球很快时直接坐下，球相对慢时进入warningstate，两个用来判断的参数分别叫v2,v3

​	**此时丢球不进入walkToSaerch直接找，因为乌龙球概率低**

#### warningstate

球动了基本都要坐下要求很低，判断用的参数叫v1

若是丢了首先进walkToSearch，防止乌龙球，然后立刻找球

#### kickball

球在禁区附近（左右1800～-1800，禁区线以后），就踢，能够包括乌龙球范围

球在1800～-1800以外且禁区线后面，进入**keepGuardLeft/Right** 

此时丢球直接进入全面找球然后再走位。

#### keepGuardLeft/RIght

球很偏时，站到定点档球，不复杂不解释了

----

### 2.3 刚刚丢球之后

我发现我好像在前面讲完了，所以这部分没啥内容，核心就是处理乌龙球机制

#### 绕球

有一个很容易出问题的是绕球，根据在场地的左右选择绕球方向，从能够挡住球门的方向绕，但是容易出问题，如果不踢球考虑一下这个
