#include <iostream>
using namespace std;
#include <ctime>
//Eigen部分
#include <Eigen/Core>
//稠密矩阵运算
#include <Eigen/Dense>
#define MATRIX_SIZE 50
/*使用方法*/
int main(int argc, char* argv[])
{
	//float Matrix
	Eigen::Matrix<float,2,3> matrix_23;
	// Vector3d 三维向量
	Eigen::Vector3d v_3d;
	Eigen::Matrix<float,3,1> vd_3d;
	//Matrix3d
	Eigen::Matrix3d matrix_33=Eigen::Matrix3d::Zero();//初始化为零
	//动态大小矩阵
	Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> matrix_dynamic;
	Eigen::MatrixXd matrix_x;
	//矩阵操作
	//初始化
	matrix_23<<1,2,3,4,5,6;
	//输出
	cout<<matrix_23<<endl;
	//()访问矩阵元素
	for (int i=0;i<2;i++){
		for (int j=0;j<3;j++)
			cout<<matrix_23(i,j)<<"\t";
		cout<<endl;
	}
        																																					//矩阵与向量相乘
	v_3d<<3,2,1;
	vd_3d<<4,5,6;
	Eigen::Matrix<double,2,1> result=matrix_23.cast<double>()*v_3d;
	cout<<result<<endl;
	Eigen::Matrix<float ,2,1> result2=matrix_23*vd_3d;
	cout<<result2<<endl;
	//不能搞错维度
	//Eigen::Matrix<double,2,3> result_wrong_dimension=matrix_23.cast<double>()*v_3d;
	//矩阵运算
	matrix_33=Eigen::Matrix3d::Random(); //随机数矩阵
	cout<<matrix_33<<endl<<endl;
	cout<<matrix_33.transpose()<<endl;   //转置
	cout<<matrix_33.sum()<<endl;         //各元素和
	cout<<matrix_33.trace()<<endl;         //迹
	cout<<10*matrix_33<<endl;             //数乘
	cout<<matrix_33.inverse()<<endl;      //逆
        cout<<matrix_33.determinant()<<endl;  //行列式
	//特征值
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_slover(matrix_33);
	cout<<"Eigen values=\n"<<eigen_slover.eigenvalues()<<endl;
	cout<<"Eigen vectors\n"<<eigen_slover.eigenvectors()<<endl;
	//解方程
	Eigen::Matrix<double,MATRIX_SIZE,MATRIX_SIZE> matrix_NN;
	matrix_NN=Eigen::MatrixXd::Random(MATRIX_SIZE,MATRIX_SIZE);
	Eigen::Matrix<double,MATRIX_SIZE,1> v_Nd;
	v_Nd=Eigen::MatrixXd::Random(MATRIX_SIZE,1);
	clock_t time_stt=clock();  //计时
	//直接求逆
	Eigen::Matrix<double,MATRIX_SIZE,1> x=matrix_NN.inverse()*v_Nd;
	cout<<"time use in nomal inverse is"<<1000*(clock()-time_stt)/(double)CLOCKS_PER_SEC<<"ms"<<endl;
	//矩阵分解求方程
	time_stt=clock();
	x=matrix_NN.colPivHouseholderQr().solve(v_Nd);
	cout<<"time use in Qr compsition is"<<1000*(clock()-time_stt)/(double)CLOCKS_PER_SEC<<"ms"<<endl;
	return 0;  
}
	
