SDL_INC=-I /* INSERT SDL INCLUDE PATH */
SDL_LNK=-L /* INSERT SDL LINK PATH */

TTF_INC=-I /* INSERT SDL_TTF INCLUDE PATH */
TTF_LNK=-L /* INSERT SDL_TTF LINK PATH */

output: src/main.o engine3d/engine3d.o simplegui/simplegui.o
	gcc src/main.o engine3d/engine3d.o simplegui/simplegui.o -o output \
	$(SDL_INC) $(TTF_INC) \
	$(SDL_LNK) $(TTF_LNK) \
	-lmingw32 -lSDL2main -lSDL2 -lSDL2_ttf

src/main.o: src/main.c constants.h
	gcc -c src/main.c -o src/main.o \
	$(SDL_INC) $(TTF_INC) \
	$(SDL_LNK) $(TTF_LNK) \
	-lmingw32 -lSDL2main -lSDL2 -lSDL2_ttf

engine3d/engine3d.o: engine3d/engine3d.c engine3d/engine3d.h constants.h
	gcc -c engine3d/engine3d.c -o engine3d/engine3d.o

simplegui/simplegui.o: simplegui/simplegui.c simplegui/simplegui.h constants.h
	gcc -c simplegui/simplegui.c -o simplegui/simplegui.o \
	$(SDL_INC) $(TTF_INC) \
	$(SDL_LNK) $(TTF_LNK) \
	-lmingw32 -lSDL2main -lSDL2 -lSDL2_ttf

clean:
	del /S *.o output

build:
	gcc -c src/main.c -Wall -o src/main.o \
	$(SDL_INC) $(TTF_INC) \
	$(SDL_LNK) $(TTF_LNK) \
	-lmingw32 -lSDL2main -lSDL2 -lSDL2_ttf \
	-O3
	gcc -c engine3d/engine3d.c -Wall -o engine3d/engine3d.o \
	-O3
	gcc -c simplegui/simplegui.c -Wall -o simplegui/simplegui.o \
	$(SDL_INC) $(TTF_INC) \
	$(SDL_LNK) $(TTF_LNK) \
	-lmingw32 -lSDL2main -lSDL2 -lSDL2_ttf \
	-O3
	gcc src/main.o engine3d/engine3d.o simplegui/simplegui.o -o build \
	$(SDL_INC) $(TTF_INC) \
	$(SDL_LNK) $(TTF_LNK) \
	-lmingw32 -lSDL2main -lSDL2 -lSDL2_ttf \
	-O3
