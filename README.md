Após compilar o ns3 no diretório <ns3_base>, faça o seguinte:

1. Clone este repositório no diretório <projeto_base>:

git clone https://github.com/lucaswnn/redes_sem_fio.git

Exemplo:

cd ~/Documentos/ns3-project

git clone https://github.com/lucaswnn/redes_sem_fio.git

2. Crie um link simbólico:

ln -s <projeto_base>/scenarios/<cenário>.cc <ns3_base>/scratch/

Exemplo:

ln -s ~/Documentos/ns3-project/scenarios/test.cc ~/ns-3-dev/scratch/

3. Rodar (diretório corrente em ns-3-dev):

./ns3 run scratch/<cenário>

Exemplo:

cd ~/ns-3-dev

.ns3 run scratch/test

ou

~/ns-3-dev/ns3 run scratch/test
