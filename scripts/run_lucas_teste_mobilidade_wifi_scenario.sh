#!/bin/bash 

NS3_PATH=~/Programas_pessoais/ns-3-dev
SCENARIO=teste_mobilidade_com_wifi

cd $NS3_PATH
./ns3 run scratch/$SCENARIO
