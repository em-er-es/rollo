Compile the library:

g++ -c -fPIC udp.cpp -o udp.o
g++ -shared -o libudp.so udp.o

then copy libudp.so in /opt/ros/hydro/lib
