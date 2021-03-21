ODE=/mnt/d/Cours/Projet/ProjFinal/ode-0.16.2-net

all : projetODE

projetODE : projetODE.o client.o creationBuggy.o deplaceBuggy.o terrain.o
	/bin/bash $(ODE)/libtool  --tag=CXX   --mode=link g++  -g -O2 -o $@ $^ $(ODE)/drawstuff/src/libdrawstuff.la $(ODE)/ode/src/libode.la -lGLU -lGL  -lrt -lm  -lpthread

%.o : %.cpp %.h
	g++ -Wall -DHAVE_CONFIG_H -I. -I$(ODE)/ode/src  -I$(ODE)/include -I$(ODE)/include -DDRAWSTUFF_TEXTURE_PATH="\"$(ODE)/drawstuff/textures\"" -DdTRIMESH_ENABLED   -g -O2 -c -o $@ $<

clean :
	rm -f *.o projetODE
