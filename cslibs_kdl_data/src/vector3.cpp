#include <cslibs_kdl_data/vector3.h>

using namespace cslibs_kdl_data;

Vector3::Vector3()
{
  zero();
}

Vector3::Vector3(double val)
{
  vector[0] = val;
  vector[1] = val;
  vector[2] = val;
}

Vector3::Vector3(double x, double y, double z)
{
  vector[0] = x;
  vector[1] = y;
  vector[2] = z;
}


void Vector3::zero()
{
  for(double& v : vector){
    v = 0;
  }
}
double& Vector3::operator ()(std::size_t i)
{
  return vector[i];
}

const double& Vector3::operator ()(std::size_t i) const
{
  return vector[i];
}

Vector3 Vector3::operator+(const Vector3 &other) const
{
  Vector3 res;
  auto it = this->vector.begin();
  auto it_o = other.vector.begin();
  for(double& r : res.vector){
    r = *it + *it_o;
    ++it;
    ++it_o;
  }
  return res;
}

Vector3 Vector3::operator-(const Vector3 &other) const
{
  Vector3 res;
  auto it = this->vector.begin();
  auto it_o = other.vector.begin();
  for(double& r : res.vector){
    r = *it - *it_o;
    ++it;
    ++it_o;
  }
  return res;
}

Vector3& Vector3::operator+=(const Vector3 &other)
{
  auto it_o = other.vector.begin();
  for(double& r : vector){
    r += *it_o;
    ++it_o;
  }
  return *this;
}

Vector3& Vector3::operator-=(const Vector3 &other)
{
  auto it_o = other.vector.begin();
  for(double& r : vector){
    r -= *it_o;
    ++it_o;
  }
  return *this;
}

Vector3 Vector3::operator*(const double &b) const
{
  Vector3 res;
  auto it = this->vector.begin();
  for(double& r : res.vector){
    r = *it *b;
    ++it;
  }

  return res;
}

Vector3 Vector3::abs() const
{
  Vector3 res;

  auto it = this->vector.begin();
  for(double& r : res.vector){
    r = std::fabs(*it);
    ++it;
  }

  return res;
}

Vector3 Vector3::operator*(const Vector3 &other) const
{
  Vector3 res;
  auto it = this->vector.begin();
  auto it_o = other.vector.begin();
  for(double& r : res.vector){
    r = *it * *it_o;
    ++it;
    ++it_o;
  }

  return res;
}

Vector3 Vector3::operator/(const double &b) const
{
  Vector3 res;

  auto it = this->vector.begin();
  for(double& r : res.vector){
    r = *it /b;
    ++it;
  }

  return res;
}

Vector3& Vector3::operator*=(const double &b)
{
  for(double& v : vector){
    v *=  b;
  }

  return *this;
}

Vector3& Vector3::operator/=(const double &b)
{

  for(double& v : vector){
    v /=  b;
  }

  return *this;
}

Vector3& Vector3::operator =(const Eigen::Vector3d &v)
{
  for(std::size_t i = 0; i < 3; ++i){
    vector[i] = v(i);
  }
  return *this;
}

double Vector3::norm() const
{
  double res = 0;
  for(auto v : vector){
    res += v*v;
  }
  res = std::sqrt(res);
  return res;
}

void Vector3::normalize()
{
  double n = norm();
  for(double& v : vector){
    v /= n;
  }
}

std::vector<double> Vector3::toVector() const
{
  std::vector<double> res = {vector[0], vector[1], vector[2]};
  return res;

}
Eigen::Vector3d Vector3::toEigen() const
{
  return Eigen::Vector3d(vector[0], vector[1], vector[2]);
}

void Vector3::fromEigen(const Eigen::Vector3d &v)
{
  for(std::size_t i = 0; i < 3; ++i){
    vector[i] = v(i);
  }
}
std::string Vector3::to_string(const std::string delimiter) const
{
  std::string res;

  for(std::size_t i = 0; i < 3; ++i)
  {
    res += std::to_string(vector[i]) + delimiter;
  }
  return res;
}
