

# 样条曲线

原A*路径：

<img src="2025-09-22 17-02-00 的屏幕截图.png" alt="2025-09-22 17-02-00 的屏幕截图" style="zoom:37%;" />

## 贝塞尔曲线

$$
B(t) = \sum_{i=0}^n C_{n}^i (1-t)^{n-i} t^i P_i \quad , \quad t \in[0,1]
$$

贝塞尔曲线平滑路径：

<img src="2025-09-22 17-00-41 的屏幕截图.png" alt="2025-09-22 17-00-41 的屏幕截图" style="zoom:50%;" />



## B样条曲线

​		   
$$
p(u) = \sum_{i=0}^{n} P_i N_{i,k}(u) \\
N_{i,0}(u) =
\begin{cases}
1, & u_i \leq u < u_{i+1} \\
0, & \text{其他情况}
\end{cases} \\
N_{i,k}(u) =
\frac{u - u_i}{u_{i+k} - u_i} N_{i,k-1}(u)
+ \frac{u_{i+k+1} - u}{u_{i+k+1} - u_{i+1}} N_{i+1,k-1}(u),
\quad u_k \leq u \leq u_{n+1}\\
N'_{i,k}(t) =
\frac{k-1}{t_{i+k-1} - t_i} N_{i,k-1}(t)
- \frac{k-1}{t_{i+k} - t_{i+1}} N_{i+1,k-1}(t)
$$

- De Boor递推算法

  

  B样条曲线平滑路径：

  <img src="2025-09-22 18-00-52 的屏幕截图-1758862141256-1.png" alt="2025-09-22 18-00-52 的屏幕截图" style="zoom:50%;" />

# 色块追踪

![output-17_59_57](output-17_59_57.gif)

**问题：**实际运行中发现深度图和RGB图像素没有一一对应，并且帧率较低。

**解决办法：**在launch文件后加入对其指令和限制图像尺寸输出。

```bash
roslaunch realsense2_camera rs_camera.launch \
    align_depth:=true \
    depth_width:=640 depth_height:=480 depth_fps:=15 \
    color_width:=640 color_height:=480 color_fps:=15
```

**深度图跟踪：**

![example2](example2.gif)

# [概率机器人](note.md)
