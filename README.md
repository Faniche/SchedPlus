# Dependencies
```shell
sudo apt install libspdlog-dev -y
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