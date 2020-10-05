#include "CarlSceneKinChar.h"

cCarlSceneKinChar::cCarlSceneKinChar()
{
}

cCarlSceneKinChar::~cCarlSceneKinChar()
{
}

void cCarlSceneKinChar::Init()
{
	bool succ = BuildCharacters();
}

void cCarlSceneKinChar::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScene::ParseArgs(parser);
	ParseCharParams(parser, mCharParams);
}

void cCarlSceneKinChar::Reset()
{
	ResetCharacters();
}

void cCarlSceneKinChar::Clear()
{
	mChar.reset();
}

void cCarlSceneKinChar::Update(double time_elapsed)
{
	UpdateCharacters(time_elapsed);
}

const std::shared_ptr<cCarlKinCharacter>& cCarlSceneKinChar::GetCharacter() const
{
	return mChar;
}

tVector cCarlSceneKinChar::GetCharPos() const
{
	return GetCharacter()->GetRootPos();
}

double cCarlSceneKinChar::GetTime() const
{
	return GetCharacter()->GetTime();
}

std::string cCarlSceneKinChar::GetName() const
{
	return "Kinematic Char";
}

void cCarlSceneKinChar::ParseCharParams(const std::shared_ptr<cArgParser>& parser, cCarlKinCharacter::tParams& out_params) const
{
	std::string char_file = "";
	std::string motion_file = "";
	std::string state_file = "";
	double init_pos_xs = 0;

	bool succ = parser->ParseString("character_file", char_file);

	if (succ)
	{
		parser->ParseString("state_file", state_file);
		parser->ParseString("motion_file", motion_file);
		parser->ParseDouble("char_init_pos_x", init_pos_xs);

		out_params.mCharFile = char_file;
		out_params.mMotionFile = motion_file;
		out_params.mStateFile = state_file;
		out_params.mOrigin[0] = init_pos_xs;
	}
	else
	{
		printf("No character file provided\n");
	}
}

bool cCarlSceneKinChar::BuildCharacters()
{
	mChar.reset();

	auto& params = mCharParams;
	params.mID = 0;
	params.mLoadDrawShapes = true;

	bool succ = BuildCharacter(params, mChar);

	return succ;
}

bool cCarlSceneKinChar::BuildCharacter(const cCarlKinCharacter::tParams& params, std::shared_ptr<cCarlKinCharacter>& out_char) const
{
	out_char = std::shared_ptr<cCarlKinCharacter>(new cCarlKinCharacter());
	bool succ = out_char->Init(params);
	return succ;
}

void cCarlSceneKinChar::ResetCharacters()
{
	mChar->Reset();
}

void cCarlSceneKinChar::UpdateCharacters(double timestep)
{
	mChar->Update(timestep);
}
