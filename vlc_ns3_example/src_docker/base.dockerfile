FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
# ENV TZ=America/Sao_Paulo

RUN apt-get update -y --fix-missing
RUN apt-get install -y  \
    build-essential     \
    cmake               \
    unzip               \
    python2             \
    python3             \
    gcc                 \
    g++                 \
    g++-multilib        \
    git                 

RUN ln -s /usr/bin/python2 /usr/bin/python

# NS3
WORKDIR /usr/src
RUN git clone https://github.com/nsnam/ns-3-dev-git.git
RUN mv ns-3-dev-git ns3
RUN cd /usr/src/ns3 &&  \
    git checkout ns-3.25

# VLC physical model
# RUN git clone https://github.com/nsnam/ns-3-dev-git.git
# ADD vlc.zip .
# RUN unzip vlc.zip && mv VLC-master /usr/src/ns3/src/vlc

WORKDIR /usr/src/ns3
RUN CXXFLAGS="-g -O0" ./waf configure
RUN CXXFLAGS="-g -O0" ./waf

