#ifndef LEAST_SQUARES_H
#define LEAST_SQUARES_H

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/Eigenvalues"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

using namespace std;

namespace path_plan{

class LeastSquares{
public:
	LeastSquares(){};
	//n-degree of equation;
	LeastSquares(int n,vector<float> v_x,vector<float> v_y,int data_num){
		N = n;
		Eigen::MatrixXd A = calc_A(v_x);//计算A矩阵
		Eigen::VectorXd B = calc_B(v_y);//计算B矩阵
		// 创建矩阵W
		Eigen::MatrixXd W;
		W = (A.transpose() * A).inverse() * A.transpose() * B;
		//cout<<W<<endl;
		//cout<<W(0)<<","<<W(1)<<","<<W(2)<<","<<W(3)<<","<<W(4)<<","<<W(5)<<endl;
		coefficient.push_back(W(0));
		coefficient.push_back(W(1));
		coefficient.push_back(W(2));
		coefficient.push_back(W(3));
		coefficient.push_back(W(4));
		coefficient.push_back(W(5));

		float temp_x,temp_y;
		for(int i = 0;i <= 100;++i ){
			temp_x = 1.0*i;
			temp_y = W(0)*pow(temp_x,5) + W(1)*pow(temp_x,4) + W(2)*pow(temp_x,3) + W(3)*pow(temp_x,2)\
					 + W(4)*pow(temp_x,1) + W(5);
			//cout<<temp_x<<","<<temp_y<<endl;
			curve_x.push_back(temp_x);
			curve_y.push_back(temp_y);
		}
	};
	float GetXValueByY(float temp_y){
		for(int i = 0;i < curve_y.size();++i){
			if(temp_y < curve_y[i])
				return curve_x[i];
		}
		return 0;
	};
private:
	Eigen::MatrixXd calc_A(vector<float> v_x){
		Eigen::MatrixXd A(v_x.size(),N + 1);
		
		for(unsigned int i = 0;i < v_x.size();++i){
			for(int n = N,dex = 0;n >= 1;--n,++dex){
				A(i,dex) = pow(v_x[i],n);
			}
			A(i,N) = 1;
		}
		return A;
	};
	Eigen::VectorXd calc_B(vector<float> v_y){
		Eigen::VectorXd B(v_y.size(),1);
		for(unsigned int i = 0; i < v_y.size(); ++i){
			B(i,0) = v_y[i];
		}
		return B;
	};
private:
	int N;
	vector<float> coefficient;
	vector<float> curve_x;
	vector<float> curve_y;
};
}

#endif


