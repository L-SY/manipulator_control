# manipulator_control

> A complete code framework for manipulator development.

目标

作为产品实现机械臂较为完善的基础功能，帮助用户更好的上手一些基础任务

- 软件上：

    - 完善的用户文档（参考https://docs.galaxea.ai/Guide/A1/Getting_Started/#next-step），简单的使用手册（快速使用）

    - 提供比较完善的官方教程，对不同研究方向的使用者都有初步的教程/示例，促进开发社区的创建。

- 机械上：
    - 需要有一个相对突出的设计亮点：例如ARX-5的Link3和星海图A1的Link2（在A1XY上也保留了），像品牌设计语言一般，其它地方保持结构上的稳定简洁即可
    - 坚持机械骨架+多样化外壳的设计（提供更加灵活的外壳设计和后期改装，包含后续的可视化灯条和柔性触觉传感器）
    - 提供六轴/七轴的选择，仅需变化一个Joint和两个Link，其它的保持不变（代码上做适配）

- 电路上：
    - 参考candelight fd开源方案(https://www.pengutronix.de/de/blog/2023-08-17-candlelight-fd-open-hardware-usb-to-can-fd-interface.html)制作usb2canfd（可以降阶当作can用）
    - 制作末端的按钮板
    - 增加灯条/触觉传感器参考https://any-skin.github.io/