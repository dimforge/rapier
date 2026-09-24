#ifndef TB_GRAPHICS_H
#define TB_GRAPHICS_H
#include "testbed.h"
#include "raylib.h"
typedef struct TbGraphics TbGraphics;
TbGraphics *tbGraphicsNew(void);
void tbGraphicsFree(TbGraphics *);
int tbGraphicsDraw(TbGraphics *, Testbed *, Camera3D, uint32_t, bool);
void tbGraphicsFrameAll(Testbed *, Camera3D *);
#endif
