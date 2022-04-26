# Dependencies
```shell
# install spdlog
sudo apt install libspdlog-dev -y
# install google test library
git clone https://github.com/google/googletest.git
mkdir /tmp/build
cmake -S googletest -B /tmp/build
cd /tmp/build && make && sudo make install
rm -r /tmp/build
```

# Topology
```shell
                +-----+               +-----+                
                | es0 |               | es2 |                
                +-----+               +-----+                
                   ^                     ^
                   |                     |
                  p0                     p0
                   v                     v
+-----+         +-----+               +-----+         +-----+
| es1 |<-----p1>| sw0 |<p2---------p2>| sw1 |<p1----->| es3 |
+-----+         +-----+               +-----+         +-----+
                   ^     \         /     ^
                  p3       +-----+       p3
                   |       | sw4 |       |
                  p2       +-----+       p3
                   v    /          \     v
+-----+         +-----+               +-----+         +-----+
| es4 |<-----p0>| sw2 |<p3---------p2>| sw3 |<p0----->| es6 |
+-----+         +-----+               +-----+         +-----+
                   ^                     ^
                  p1                     p1
                   |                     |
                   v                     v
                +-----+               +-----+                
                | es5 |               | es7 |                
                +-----+               +-----+      
```

流类型	优先级	周期	时延要求	帧长/B
同步实时	7			固定 30~100
周期循环	6			固定 50~1 000
音频	5			可变 1 000~1 500
视频	4			可变 1 000~1 500
网络控制	3			可变 50~500
配置&诊断	2			可变 500~1 500
Best Effort	1			可变 30~1 500
事件	0			可变 100~1 500

Init totally token: 31030082 ms.
ga_obj.population = 100;
ga_obj.generation_max = 30;
200     603.003s


Init totally token: 418895094 ms.
1000    34956s