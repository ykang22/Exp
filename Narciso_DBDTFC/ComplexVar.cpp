// MotionObserver.cpp
// Marc Petit
// 04/07/15 
# include "ComplexVar.h"

/*
	Input:
*/
// ComplexVar::ComplexVar ()
// {
	// d = 0.f;
	// q = 0.f;
// }

// ComplexVar::ComplexVar (float d_comp, float q_comp)
// {
	// d = d_comp;
	// q = q_comp;
// }

// ComplexVar ComplexVar::operator+(const ComplexVar& dq)
// {
	// ComplexVar dq_out;
	// dq_out.d = this->d + dq.d;
	// dq_out.q = this->q + dq.q;
	// return dq_out;
// }

// ComplexVar ComplexVar::operator-(const ComplexVar& dq)
// {
	// ComplexVar dq_out;
	// dq_out.d = this->d - dq.d;
	// dq_out.q = this->q - dq.q;
	// return dq_out;
// }

// ComplexVar ComplexVar::operator*(const ComplexVar& dq)
// {
	// ComplexVar dq_out;
	// dq_out.d = this->d * dq.d - this->q * dq.q;
	// dq_out.q = this->d * dq.q + this->q * dq.d;
	// return dq_out;
// }
// ComplexVar ComplexVar::operator*(const float& scalar)
// {
	// ComplexVar dq_out;
	// dq_out.d = this->d * scalar;
	// dq_out.q = this->q * scalar;
	// return dq_out;
// }

// ComplexVar ComplexVar::operator+=(const ComplexVar& dq)
// {
	// this->d += dq.d;
	// this->q += dq.q;
	// return (*this);
// }
// ComplexVar ComplexVar::scal_mul(const float& factor)
// {
	// ComplexVar dq_out;
	// dq_out.d = this->d * factor;
	// dq_out.q = this->q * factor;
	// return dq_out;
// }


// ComplexVar ComplexVar::comp_mul(const ComplexVar& dq)
// {
	// ComplexVar dq_out;
	// dq_out.d = this->d * dq.d;
	// dq_out.q = this->q * dq.q;
	// return dq_out;
// }

//// ab -> dq : theta > 0
// //dq -> ab: theta > 0
// ComplexVar ComplexVar::rotate(const float& theta)
// {
	// ComplexVar dq_out;
	// dq_out.d = this->d * cosf(theta) - this->q * sinf(theta);
	// dq_out.q = this->d * sinf(theta) + this->q * cosf(theta);
	// return dq_out;
// }

// float ComplexVar::real(const ComplexVar& dq)
// {
	// return dq.d;
// }

// float ComplexVar::imag(const ComplexVar& dq)
// {
	// return dq.q;
// }

// void ComplexVar::clear()
// {
	// this->d = 0;
	// this->q = 0;
// }

// float ComplexVar::get_d_comp()
// {
	// return d;
// }

// float ComplexVar::get_q_comp()
// {
	// return q;
// }


