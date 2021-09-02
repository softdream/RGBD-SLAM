#include <iostream>
#include <Eigen/Core>
#include <sophus/so3.h>
#include <sophus/se3.h>

using namespace std;
//using namespace Eigen;
//using namespace Sophus;


int main(int argc, char **argv) {
    //����Z����ת90�ȵ���ת����
    Eigen::AngleAxisd A1(M_PI / 2, Eigen::Vector3d(0, 0, 1));//�ԣ�0,0,1��Ϊ��ת�ᣬ��ת180��
    Eigen::Matrix3d R1 = A1.matrix();
    Eigen::Quaterniond Q1(A1);


    //һ����ʼ������Ⱥ��SO3���ļ��ַ�ʽ

    //1.ʹ����ת�����ʼ����Ⱥ
    Sophus::SO3 SO3_R(R1);
    //ע�⣺����SO(3)�Ƕ�Ӧһ������,�������SO(3)ʱ,ʵ��������so(3)��ʽ���,������Ľ�����Կ���,�������ֵ����ת�Ƕ�Ӧ��ֵ��ͬ,��Ҳ֤֤ʵ��SO(3)��Ӧ�������so(3)������ת�ǡ�
    cout << "SO(3) SO3_R from Matrix" << SO3_R << endl << endl;

    //2.ʹ����Ԫ����ʼ����Ⱥ
    Sophus::SO3 SO3_Q(Q1);
    cout << "SO(3) SO3_Q from Quaterion" << SO3_Q << endl << endl;

    /****************************************************************************
     3.1 ʹ����ת�ǣ���ǣ��ĸ���Ԫ�ض�Ӧ�Ĵ���ֵ����ʼ����Ⱥ

     ע�⣺ֱ��ʹ����ת��AngleAxis������ת�Ƕȶ�Ӧ������(Vector3d=AngleAxis.axis()*AngleAxis.angle())����Ⱥ���г�ʼ���ǲ��еģ���ΪSO3��Ⱥû�ж�Ӧ�Ĺ��캯����
    Ҳ����ʹ�����з����Ǵ���ģ�

     Sophus::SO3 SO3_A(A1);//ֱ��ʹ����ת�Ƕ���Ⱥ��ʼ��
     Sophus::SO3 SO3_A(A1.axis()*A1.angle());//ֱ��ʹ����ת�Ƕȶ�Ӧ������(Vector3d=AngleAxis.axis()*AngleAxis.angle())����Ⱥ���г�ʼ��

     ֻ��ʹ����ת�Ƕ�Ӧ��������ÿһ��ά�Ƚ��и�ֵ����Ӧ��SO3������һ�����캯��SO3(double rot_x, double rot_y, double rot_z);

    *******************************************************************************/

    //3.1.1 ʹ����ת�Ƕȶ�Ӧ������(Vector3d=AngleAxis.axis()*AngleAxis.angle())�еĸ���Ԫ�ض���Ⱥ���г�ʼ��
    Sophus::SO3 SO3_A1((A1.axis() * A1.angle())(0), (A1.axis() * A1.angle())(1), (A1.axis() * A1.angle())(2));
    cout << "SO(3) SO3_A1 from AngelAxis1" << SO3_A1 << endl << endl;

    //3.1.2 ʹ����ת�Ƕȶ�Ӧ������(Vector3d=AngleAxis.axis()*AngleAxis.angle())�еĸ���Ԫ�ض���Ⱥ���г�ʼ��
    Sophus::SO3 SO3_A2(M_PI / 2 * 0, M_PI / 2 * 0, M_PI / 2 * 1);
    cout << "SO(3) SO3_A2 from AngleAixs2" << SO3_A2 << endl << endl;

    //3.2 ������ת�ǣ���ǣ��������so(3)��Ӧ,����ֱ��ʹ����ת�ǵ�ֵ���se(3),������ͨ��Sophus::SO3::exp()��ö�Ӧ��SO(3)
    Eigen::Vector3d V1(0, 0, M_PI / 2);//so3��Eigen����Vector3d��ʾ
    Sophus::SO3 SO3_V1 = Sophus::SO3::exp(V1);
    cout << "SO(3) SO3_V1 from SO3::exp()" << SO3_V1 << endl << endl;


    //����SO(3)��so(3)���໥ת�����Լ�so3��Ӧ��hat��vee����

    Eigen::Vector3d so3_V1 = SO3_V1.log();//so(3)��Sophus(Eigen)����vector3d��ʾ,ʹ�ö���ӳ������Ⱥ��Ӧ�������
    cout << "so(3) so3_V1 from SO3_V1" << so3_V1.transpose() << endl << endl;


    Sophus::SO3 SO3_V2 = Sophus::SO3::exp(so3_V1);//ʹ��ָ��ӳ�佫�����ת��Ϊ��Ⱥ
    cout << "SO(3) so3_V2 from so3_V1" <<SO3_V2 << endl << endl;


    Eigen::Matrix3d M_so3_V1 = Sophus::SO3::hat(so3_V1);//hatΪ���������Ӧ�ķ��Գƾ���
    cout << "so3 hat=\n" << M_so3_V1 << endl << endl;

    Eigen::Vector3d V_M = Sophus::SO3::vee(M_so3_V1);//veeΪ���Գƾ����Ӧ������
    cout << "so3 vee=\n" << V_M << endl << endl;

    //���������Ŷ�ģ��
    Eigen::Vector3d update_so3(1e-4,0,0);//���������Ϊ��ô��
    Eigen::Matrix3d update_matrix=Sophus::SO3::exp(update_so3).matrix();//����Ⱥת��Ϊ��ת����
    cout<<"SO3 update Matrix=\n"<<update_matrix<<endl<<endl;

    Sophus::SO3 SO3_updated=Sophus::SO3::exp(update_so3)*SO3_R;
    cout<<"SO3 updated = \n"<<SO3_updated<<endl;

    Eigen::Matrix3d SO3_updated_matrix=SO3_updated.matrix();//����Ⱥת��Ϊ��ת����
    cout<<"SO3 updated Matrix = \n"<<SO3_updated_matrix<<endl<<endl;


//******************************************************************�ָ���***********************************************************************************
    cout<<"************************************ SPLIT *************************************************"<<endl<<endl;

    Eigen::AngleAxisd A2(M_PI/2,Eigen::Vector3d(0,0,1));
    Eigen::Matrix3d R2=A2.matrix();
    Eigen::Quaterniond Q2(A2);
    Sophus::SO3 SO3_2(R2);

    //һ����ʼ��������ļ��ַ�ʽ
    Eigen::Vector3d t(1,0,0);

    //1. ʹ����ת�����ƽ����������ʼ��SE3
    Sophus::SE3 SE_Rt(R2,t);
    cout<<"SE3 SE_Rt from  Rotation_Matrix and Transform=\n"<<SE_Rt<<endl<<endl;//ע�⾡��SE(3)�Ƕ�Ӧһ��4*4�ľ���,�������SE(3)ʱ�????һ����ά���������,����ǰǰ��λΪ��Ӧ��so3,��3ά��Ϊʵ�??��ƽ����t��������se3�е�ƽ�Ʒ���
    //2. ʹ����Ԫ����ƽ���??������ʼ��SE3
    Sophus::SE3 SE_Qt(Q2,t);
    cout<<"SE3 SE_Qt from  Quaterion and Transform=\n"<<SE_Qt<<endl<<endl;
    //3. ʹ��SO3��ƽ����������ʼ��SE3
    Sophus::SE3 SE_St(SO3_2,t);
    cout<<"SE3 SE_St from  SO3 and Transform=\n"<<SE_St<<endl<<endl;

    //����SE(3)��se(3)���໥ת�����Լ�se3��Ӧ��hat�??vee����
    Sophus::Vector6d se3_Rt=SE_Rt.log();//se(3)��Sophus����Vector6d��ʾ,ʹ�ö���ӳ������Ⱥ��Ӧ�������
    cout<<"se(3) se3_Rt from SE3_Rt\n"<<se3_Rt<<endl<<endl;//se3��������????��ά������,����ǰ3ά��ƽ�Ʒ���,��3ά������ת����

    Sophus::SE3 SE3_Rt2=Sophus::SE3::exp(se3_Rt);//ʹ��ָ��ӳ�佫������??��Ϊ��Ⱥ
    cout<<"SE(3) SO3_Rt2 from se3_Rt"<<SE3_Rt2<<endl<<endl;

    Sophus::Matrix4d M_se3_Rt=Sophus::SE3::hat(se3_Rt);
    cout<<"se(3) hat=\n"<<M_se3_Rt<<endl<<endl;

    Sophus::Vector6d V_M_se3=Sophus::SE3::vee(M_se3_Rt);
    cout<<"se(3) vee=\n"<<V_M_se3<<endl<<endl;




    //���������Ŷ�ģ��

    Sophus::Vector6d update_se3=Sophus::Vector6d::Zero();
    update_se3(0)=1e-4d;

    cout<<"update_se3\n"<<update_se3.transpose()<<endl<<endl;

    Eigen::Matrix4d update_matrix2=Sophus::SE3::exp(update_se3).matrix();//����Ⱥת��Ϊ��ת����
    cout<<"update matrix=\n"<<update_matrix2<<endl<<endl;

    Sophus::SE3 SE3_updated=Sophus::SE3::exp(update_se3)*SE3_Rt2;
    cout<<"SE3 updated=\n"<<SE3_updated<<endl<<endl;

    Eigen::Matrix4d SE3_updated_matrix=SE3_updated.matrix();//����Ⱥת��Ϊ��ת����
    cout<<"SE3 updated Matrix=\n"<<SE3_updated_matrix<<endl<<endl;

    return 0;

}
