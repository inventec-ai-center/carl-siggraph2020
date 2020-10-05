#include "DrawCarlSceneKinChar.h"

#include "render/DrawUtilExtend.h"
#include "render/DrawCharacterExtend.h"
#include "util/FileUtil.h"
#include "anim/CarlKinTree.h"

extern int g_ScreenWidth;
extern int g_ScreenHeight;

const double gLinkWidth = 0.025f;
const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gFilLColor = tVector(0.6f, 0.65f, 0.675f, 1);
const tVector gCamFocus0 = tVector(0, 0.75, 0, 0);
const double gGroundHeight = 0;

#define GAIT_PATTERN_BUFFER_SIZE 100

cDrawCarlSceneKinChar::cDrawCarlSceneKinChar() : cDrawScene()
{
    mEnableDrawGaitPattern = true;

    mColors.clear();
    mColors.push_back(tVector(0, 0, 1, 0.5));
	mColors.push_back(tVector(1, 0, 0, 0.5));
	mColors.push_back(tVector(0, 0.5, 0, 0.5));
	mColors.push_back(tVector(0.75, 0, 0.75, 0.5));
	mColors.push_back(tVector(0, 0.5, 0.5, 0.5));
	mColors.push_back(tVector(0, 0, 0, 0.5));
}

cDrawCarlSceneKinChar::~cDrawCarlSceneKinChar()
{
}

void cDrawCarlSceneKinChar::Init()
{
	BuildScene(mScene);
	SetupScene(mScene);

	cDrawScene::Init();
    InitGaits();
}

void cDrawCarlSceneKinChar::Reset()
{
	cDrawScene::Reset();
	mScene->Reset();
}

void cDrawCarlSceneKinChar::Clear()
{
	cDrawScene::Clear();
	mScene->Clear();
}

void cDrawCarlSceneKinChar::Update(double time_elapsed)
{
	cDrawScene::Update(time_elapsed);
	UpdateScene(time_elapsed);
	UpdateCamera();

    if (mEnableDrawGaitPattern)
	{
		UpdateGaits();
	}
}

void cDrawCarlSceneKinChar::Keyboard(unsigned char key, double device_x, double device_y)
{
	switch (key)
	{
	case 'u':
		ToggleGaitPattern();
		break;
	case 'k':
		ToggleDrawCharacterBody();
		break;
	default:
        cDrawCarlScene::Keyboard(key, device_x, device_y);
		break;
	}
}

std::string cDrawCarlSceneKinChar::GetName() const
{
	return mScene->GetName();
}

void cDrawCarlSceneKinChar::BuildScene(std::shared_ptr<cCarlSceneKinChar>& out_scene) const
{
	out_scene = std::shared_ptr<cCarlSceneKinChar>(new cCarlSceneKinChar());
}

void cDrawCarlSceneKinChar::SetupScene(std::shared_ptr<cCarlSceneKinChar>& out_scene)
{
	out_scene->ParseArgs(mArgParser);
	out_scene->Init();
}

void cDrawCarlSceneKinChar::UpdateScene(double time_elapsed)
{
	mScene->Update(time_elapsed);
}

tVector cDrawCarlSceneKinChar::GetCamTrackPos() const
{
	return mScene->GetCharPos();
}

tVector cDrawCarlSceneKinChar::GetCamStillPos() const
{
	return mScene->GetCharPos();
}

tVector cDrawCarlSceneKinChar::GetDefaultCamFocus() const
{
	return gCamFocus0;
}

void cDrawCarlSceneKinChar::DrawGround() const
{
	tVector ground_col = GetGroundColor();
	cDrawUtil::SetColor(ground_col);
	DrawGround3D();
}

void cDrawCarlSceneKinChar::DrawGround3D() const
{
	const double w = 200;
	const tVector ground_origin = tVector(0, gGroundHeight, 0, 0);
	const tVector tex_size = tVector(0.5, 0.5, 0, 0);

	const auto& character = mScene->GetCharacter();
	tVector char_pos = character->GetRootPos();
	char_pos[1] = gGroundHeight;
	tVector a = char_pos - tVector(-0.5 * w, 0, -0.5 * w, 0);
	tVector b = char_pos - tVector(-0.5 * w, 0, 0.5 * w, 0);
	tVector c = char_pos - tVector(0.5 * w, 0, 0.5 * w, 0);
	tVector d = char_pos - tVector(0.5 * w, 0, -0.5 * w, 0);

	tVector min_coord = a - ground_origin;
	tVector max_coord = c - ground_origin;
	min_coord[0] /= tex_size[0];
	min_coord[1] = min_coord[2] / tex_size[1];
	max_coord[0] /= tex_size[0];
	max_coord[1] = max_coord[2] / tex_size[1];

	tVector coord_a = tVector(min_coord[0], min_coord[1], 0, 0);
	tVector coord_b = tVector(min_coord[0], max_coord[1], 0, 0);
	tVector coord_c = tVector(max_coord[0], max_coord[1], 0, 0);
	tVector coord_d = tVector(max_coord[0], min_coord[1], 0, 0);

	cDrawUtil::DrawQuad(a, b, c, d, coord_a, coord_b, coord_c, coord_d);
}

void cDrawCarlSceneKinChar::DrawCharacters() const
{
	const auto& character = mScene->GetCharacter();

	if (mEnableDrawCharacterBody)
	{
		cDrawCharacter::Draw(*character, gLinkWidth, gFilLColor, gLineColor);
	}
    cDrawCharacterExtend::DrawBones(*character, 0.03f, tVector(1, 0.75, 0.25, 0.5), tVector(1, 0, 0, 1));
}

void cDrawCarlSceneKinChar::DrawInfo() const
{
	DrawInfoText();
    if (mEnableDrawGaitPattern)
	{
		DrawInfoGaitPattern();
	}
}

void cDrawCarlSceneKinChar::DrawInfoText() const
{
    const auto& character = mScene->GetCharacter();

	cDrawUtilExtend::BeginDrawString();
	{
		char str[64];
		float screen_y = g_ScreenHeight * 0.95;
		float screen_x = g_ScreenHeight * 0.02;
		sprintf(str, "Phase: %.3f\nMotion Time: %.1f s", character->GetPhase(), character->GetPhase() * character->GetMotionDuration());
		cDrawUtilExtend::DrawString(screen_x, screen_y, str);
	}
	cDrawUtilExtend::EndDrawString();
}

void cDrawCarlSceneKinChar::DrawMisc() const
{
	if (mEnableDrawGaitPattern)
	{
		DrawEndEffectors();
	}

	cDrawUtilExtend::DrawXyzAxes();
}

void cDrawCarlSceneKinChar::DrawInfoGaitPattern() const
{
	int num_val = static_cast<int>(mGaits[0].size());
	double aspect = mCamera.GetAspectRatio();

	const double h = 0.3;
	const double w = 1.94;
	tVector origin = tVector::Zero();
	origin[0] = -0.97;
	origin[1] = -0.95;
	origin[2] = -1;

	int capacity = GAIT_PATTERN_BUFFER_SIZE;

	cDrawUtil::SetLineWidth(1);
	cDrawUtil::SetColor(tVector(1, 1, 1, 0.5));
	cDrawUtil::DrawRect(origin + 0.5 * tVector(w, h, 0, 0), tVector(w, h, 0, 0));
	cDrawUtil::SetColor(tVector(0, 0, 0, 1));
	cDrawUtil::DrawRect(origin + 0.5 * tVector(w, h, 0, 0), tVector(w, h, 0, 0), cDrawUtil::eDrawWireSimple);

	cDrawUtil::SetLineWidth(1);
	cDrawUtil::SetPointSize(2);
	cDrawUtil::SetColor(tVector(1, 0, 0, 0.5));

	const double h_step = h / (double)mGaits.size();

	if (num_val > 0)
	{
		for (int i = 0; i < mGaits.size(); ++i)
		{
			if (i > 0)
			{
				cDrawUtil::SetColor(tVector(0, 0, 0, 0.5));
				cDrawUtil::DrawLine(origin + tVector(0, h_step * i, 0, 0), origin + tVector(w, h_step * i, 0, 0));
			}

			cDrawUtil::SetColor(mColors[i]);

			double prev_val = mGaits[i][0] ? 1 : 0;

			int rect_start_j = -1;
			if (prev_val > 0)
			{
				rect_start_j = 0;
			}

			for (int j = 1; j < num_val; ++j)
			{
				double curr_val = mGaits[i][j] ? 1 : 0;

				if (rect_start_j >= 0 && ((curr_val != prev_val) || (j == num_val - 1)))
				{
					tVector rect_origin = tVector::Zero();
					tVector rect_size = tVector::Zero();

					rect_size[0] = w * (j - rect_start_j) / (capacity - 1.0);
					rect_size[1] = h_step;

					rect_origin[0] = w * rect_start_j / (capacity - 1.0) + rect_size[0] * 0.5;
					rect_origin[1] = h_step * i + h_step - rect_size[1] * 0.5;
					rect_origin += origin;

					cDrawUtil::DrawRect(rect_origin, rect_size);

					rect_start_j = -1;
				}
				else if (rect_start_j < 0 && curr_val != prev_val)
				{
					rect_start_j = j;
				}

				prev_val = curr_val;
			}
		}
	}
}

void cDrawCarlSceneKinChar::UpdateGaits()
{
	const std::shared_ptr<cKinCharacter> kin_char = mScene->GetCharacter();
	const Eigen::MatrixXd& joint_mat = kin_char->GetJointMat();
	const Eigen::MatrixXd& pose = kin_char->GetPose();
	for (int i = 0; i < mGaits.size(); ++i)
	{
		int joint_id = mGaitJoints[i];
		double thres = cCarlKinTree::GetContactThreshold(joint_mat, joint_id);
		tVector offset = cCarlKinTree::GetContactOffset(joint_mat, joint_id);
		tMatrix m = cCarlKinTree::JointWorldTrans(joint_mat, pose, joint_id);
		tVector pos_contact = m * offset;

		bool contact = pos_contact[1] < thres;
		mGaits[i].push_back(contact);
		if (mGaits[i].size() > GAIT_PATTERN_BUFFER_SIZE)
		{
			mGaits[i].pop_front();
		}
	}
}

void cDrawCarlSceneKinChar::ToggleGaitPattern()
{
	mEnableDrawGaitPattern = !mEnableDrawGaitPattern;
	if (mEnableDrawGaitPattern)
	{
		InitGaits();
		printf("Enable Drawing Gait Pattern\n");
	}
	else
	{
		printf("Disable Drawing Gait Pattern\n");
	}
}

void cDrawCarlSceneKinChar::ToggleDrawCharacterBody()
{
	mEnableDrawCharacterBody = !mEnableDrawCharacterBody;
	if (mEnableDrawCharacterBody)
	{
		printf("Enable Drawing Character Body\n");
	}
	else
	{
		printf("Disable Drawing Character Body\n");
	}
}

void cDrawCarlSceneKinChar::InitGaits()
{
	mGaits.clear();
	mGaitJoints.clear();

	const std::shared_ptr<cKinCharacter> kin_char = mScene->GetCharacter();
	const Eigen::MatrixXd& joint_mat = kin_char->GetJointMat();
	for (int i = 0; i < kin_char->GetNumJoints(); ++i)
	{
		if (cCarlKinTree::IsFootContact(joint_mat, i))
		{
			mGaits.push_back(std::deque<bool>());
			mGaitJoints.push_back(i);
		}
	}
}

void cDrawCarlSceneKinChar::DrawEndEffectors() const
{
	const std::shared_ptr<cCarlKinCharacter> kin_char = mScene->GetCharacter();
	const Eigen::MatrixXd& joint_mat = kin_char->GetJointMat();
	const Eigen::MatrixXd& pose = kin_char->GetPose();
	for (int i = 0; i < mGaits.size(); ++i)
	{
		int joint_id = mGaitJoints[i];
		double thres = cCarlKinTree::GetContactThreshold(joint_mat, joint_id);
		tVector offset = cCarlKinTree::GetContactOffset(joint_mat, joint_id);
		tMatrix m = cCarlKinTree::JointWorldTrans(joint_mat, pose, joint_id);
		tVector pos_contact = m * offset;

		tVector col = mColors[i];
		cDrawUtil::PushMatrixView();
		cDrawUtil::SetColor(col);
		cDrawUtil::Translate(pos_contact);
		cDrawUtil::DrawSphere(thres);
		cDrawUtil::PopMatrixView();
	}
}