#pragma once

#include "scenes/DrawScene.h"

class cDrawCarlScene: virtual public cDrawScene
{
public:

    ~cDrawCarlScene();
    virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);

protected:

    cDrawCarlScene();
    virtual void ParseCamView(const std::shared_ptr<cArgParser>& parser);
    virtual void Reshape(int w, int h);
    virtual void InitCamera();

    tVector mCamPosition;
	tVector mCamUp;
	double mCamZoom;  
};
