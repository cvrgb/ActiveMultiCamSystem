# Active Multi-Cam System

## 名字规范

产品分三大类

1. 经典仿生眼 BE = Bionic Eyes
2. 小型并联眼 PE = Parallel Eyes
3. 鹰眼 EE = Eagle Eyes

## 配置文件结构

```
├─ "BE101L"
│   ├─ "K"
│   │   ├─ 0: double [9]
│   │   ├─ 1: double [9]
│   │   └─ 2: double [9]
│   ├─ "D"
│   │   ├─ 0: double [5 or 8]
│   │   ├─ 1: double [5 or 8]
│   │   └─ 2: double [5 or 8]
│   ├─ "H"
│   │   ├─ 1_0: double [9]
│   │   └─ 2_1: double [9]
│   ├─ "T"
│   │   ├─ 1_0: double [16]
│   │   └─ 2_1: double [16]
│   ├─ "X"
│   │   ├─ 0: double [16]
│   │   ├─ 1: double [16]
│   │   └─ 2: double [16]
│   │
├─ "BE101R"
│   ├─ "K"
│   │   ├─ 0: double [9]
│   │   ├─ 1: double [9]
│   │   └─ 2: double [9]
│   ├─ "D"
│   │   ├─ 0: double [5 or 8]
│   │   ├─ 1: double [5 or 8]
│   │   └─ 2: double [5 or 8]
│   ├─ "H"
│   │   ├─ 0_1: double [9]
│   │   └─ 1_2: double [9]
│   ├─ "T"
│   │   ├─ 0_1: double [16]
│   │   └─ 1_2: double [16]
│   ├─ "X"
│   │   ├─ 0: double [16]
│   │   ├─ 1: double [16]
│   │   └─ 2: double [16]
│   │
├─ "BE101R_BE101L"
│   ├─ "T": double [16]
```

## 模组

1. 单目多目标定
2. 手轴眼标定
3. 正反运动学
4. 通用算法/工具
