FROM ns3:3.25

WORKDIR /vlc
ADD vlc.tar .
# RUN tar -xf vlc.tar && cp -rf vlc /usr/src/ns3/src/
RUN cp -rf vlc /usr/src/ns3/src/

WORKDIR /usr/src/ns3
RUN CXXFLAGS="-g -O0" ./waf configure
RUN CXXFLAGS="-g -O0" ./waf

RUN cp /usr/src/ns3/src/vlc/examples/vlc-example.cc /usr/src/ns3/scratch
RUN ./waf --run vlc-example