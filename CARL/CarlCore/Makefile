BIN=CarlMain
OBJ_DIR = objs
MKDIR = mkdir -p

DEEPMIMIC_DIR = ../../DeepMimic/DeepMimicCore

EIGEN_DIR = /usr/local/include/eigen3
BULLET_INC_DIR = /usr/local/include/bullet

PYTHON_INC = /home/aic/miniconda3/envs/env_carl/include/python3.7m
PYTHON_LIB = /home/aic/miniconda3/envs/env_carl/lib/ -lpython3.7m

INC = -I./ \
	-I$(DEEPMIMIC_DIR) \
	-I$(EIGEN_DIR) \
	-I$(BULLET_INC_DIR)

LIBS = -lGLEW -lGL -lGLU -lglut -lBulletDynamics -lBulletCollision -lLinearMath -lm -lstdc++

CC = clang++
CFLAGS = -std=c++11 -O3 -Wall -fPIC

DEEPMIMIC_SRC_DIRS = $(DEEPMIMIC_DIR)/util/ \
					 $(DEEPMIMIC_DIR)/util/json/ \
					 $(DEEPMIMIC_DIR)/anim/ \
					 $(DEEPMIMIC_DIR)/sim/ \
					 $(DEEPMIMIC_DIR)/render/ \
					 $(DEEPMIMIC_DIR)/render/lodepng/ \
					 $(DEEPMIMIC_DIR)/scenes/ \
					 $(DEEPMIMIC_DIR)/scenes/arm/ \
					 $(DEEPMIMIC_DIR)/scenes/particle/ \
					 $(DEEPMIMIC_DIR)/scenes/pendulum/

CARL_SRC_DIRS = scenes/ \
				util/ \
				anim/ \
				sim/ \
				render/

DEEPMIMIC_SRC_FILES = $(DEEPMIMIC_DIR)/DeepMimicCore.cpp \
					  $(foreach dir, $(DEEPMIMIC_SRC_DIRS), $(wildcard $(dir)*.cpp))

CARL_SRC_FILES = $(BIN).cpp \
				 CarlCore.cpp \
				 $(foreach dir, $(CARL_SRC_DIRS), $(wildcard $(dir)*.cpp))

OUT_DIRS = $(patsubst $(DEEPMIMIC_DIR)%, $(OBJ_DIR)%, $(DEEPMIMIC_SRC_DIRS)) objs/render/OpenGLText/

OBJ = $(patsubst $(DEEPMIMIC_DIR)%, $(OBJ_DIR)%, $(patsubst %.cpp, %.o, $(DEEPMIMIC_SRC_FILES))) \
	  $(addprefix $(OBJ_DIR)/, $(patsubst %.cpp, %.o, $(CARL_SRC_FILES)))


default: build

build: $(BIN)

print:
	echo $(OUT_DIRS)
	echo $(OBJ)

python: $(OUT_DIRS) $(OBJ)
	swig -c++ -python CarlCore.i
	$(CC) -c -g $(CFLAGS) $(INC) $(LIBS) CarlCore.cpp CarlCore_wrap.cxx -I$(PYTHON_INC)
	$(CC) -shared $(CFLAGS) $(OBJ) $(LIBS) CarlCore_wrap.o -o _CarlCore.so -L$(PYTHON_LIB)

$(OBJ_DIR)/%.o: $(DEEPMIMIC_DIR)/%.cpp
	$(CC) -c -g $(CFLAGS) $(INC) -o $@ $<

$(OBJ_DIR)/%.o: %.cpp
	$(CC) -c -g $(CFLAGS) $(INC) -o $@ $<

$(BIN): $(OUT_DIRS) $(OBJ)
	$(CC) -g $(CFLAGS) $(INC) $(OBJ) $(LIBS) -o $@

$(OUT_DIRS):
	$(MKDIR) $(OUT_DIRS)

clean:
	- rm -f -r $(OUT_DIRS) $(BIN) $(OBJ) $(OBJ_DIR) CarlCore_wrap.cxx _CarlCore.so
