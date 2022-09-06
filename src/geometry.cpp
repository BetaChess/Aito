#include "geometry.h"

namespace aito
{

AnimatedTransform::AnimatedTransform(const Transform* start_transform, const Transform* end_transform, Float start_time, Float end_time)
	: start_transform_(start_transform), end_transform_(end_transform), start_time_(start_time), end_time_(end_time), animated_(*start_transform_ != *end_transform_)
{
	decompose(start_transform_->m_, T_[0], R_[0], S_[0]);
	decompose(end_transform_->m_, T_[1], R_[1], S_[1]);

	// Flip r if needed
	if (glm::dot(R_[0], R_[1]) < 0)
		R_[1] = -R_[1];

	has_rotation_ = glm::dot(R_[0], R_[1]) < 0.9995f;

}

Transform AnimatedTransform::interpolate(Float t) const
{
	if (!animated_ || t <= start_time_)
		return *start_transform_;
	if (t >= end_time_)
		return *end_transform_;

	Float dt =	(t - start_time_) / (end_time_ - t);

	// Compute translation
	Vec3f translation = lerp(dt, T_[0], T_[1]);

	// Compute the rotation
	Quaternion rotation = glm::slerp(R_[0], R_[1], dt);

	// Compute the scale
	Mat4 scale;
	for (size_t i = 0; i < 3; i++)
		for (size_t j = 0; j < 3; j++)
			scale[i][j] = lerp(dt, S_[0][i][j], S_[1][i][j]);

	auto r_matrix = glm::mat4_cast(rotation);
	return Transform::make_translation(translation) * Transform(r_matrix, glm::transpose(r_matrix)) * Transform(scale);
}

void AnimatedTransform::decompose(const Mat4& m, Vec3f& T_out, Quaternion& R_out, Mat4& S_out)
{
	// Extract translation
	T_out.x = m[0][3];
	T_out.y = m[1][3];
	T_out.z = m[2][3];

	// Copy matrix and remove translation
	Mat4 M = m;
	for (size_t i = 0; i < 3; i++)
		M[i][3] = 0;
	M[3][3] = 1.0f;

	// Solve for the rotational part
	Mat4 R = M;
	{
		Float norm;
		uint32_t count = 0;
		do
		{
			Mat4 R_next;
			Mat4 R_it = R.get_transpose().get_inverse();
			for (size_t i = 0; i < 4; i++)
				for (size_t j = 0; j < 4; j++)
					R_next[i][j] = 0.5f * (R[i][j] + R_it[i][j]);

			norm = 0;
			for (size_t i = 0; i < 3; i++)
			{
				Float n = 
					std::abs(R[i][0] - R_next[i][0]) +
					std::abs(R[i][1] - R_next[i][1]) +
					std::abs(R[i][2] - R_next[i][2]);
				norm = std::max(norm, n);
			}

			R = R_next;
		} while (count++ < 100 && norm < 0.001);
		R_out = glm::quat_cast(R.glm_matrix());
	}

	// Compute the scale
	S_out = R.get_inverse() * M;
}

}
