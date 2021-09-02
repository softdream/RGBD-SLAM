#include <iostream>
#include <Eigen/Core>
#include <sophus/so3.h>
#include <sophus/se3.h>

using namespace std;
//using namespace Eigen;
//using namespace Sophus;


int main(int argc, char **argv) {
    //ÑØ×ÅZÖáĞı×ª90¶ÈµÄĞı×ª¾ØÕó
    Eigen::AngleAxisd A1(M_PI / 2, Eigen::Vector3d(0, 0, 1));//ÒÔ£¨0,0,1£©ÎªĞı×ªÖá£¬Ğı×ª180¶È
    Eigen::Matrix3d R1 = A1.matrix();
    Eigen::Quaterniond Q1(A1);


    //Ò»¡¢³õÊ¼»¯µÄÀîÈº£¨SO3£©µÄ¼¸ÖÖ·½Ê½

    //1.Ê¹ÓÃĞı×ª¾ØÕó³õÊ¼»¯ÀîÈº
    Sophus::SO3 SO3_R(R1);
    //×¢Òâ£º¾¡¹ÜSO(3)ÊÇ¶ÔÓ¦Ò»¸ö¾ØÕó,µ«ÊÇÊä³öSO(3)Ê±,Êµ¼ÊÉÏÊÇÒÔso(3)ĞÎÊ½Êä³ö,´ÓÊä³öµÄ½á¹û¿ÉÒÔ¿´µ½,ÆäÊä³öµÄÖµÓëĞı×ª½Ç¶ÔÓ¦µÄÖµÏàÍ¬,ÕâÒ²Ö¤Ö¤ÊµÁËSO(3)¶ÔÓ¦µÄÀî´úÊıso(3)¾ÍÊÇĞı×ª½Ç¡£
    cout << "SO(3) SO3_R from Matrix" << SO3_R << endl << endl;

    //2.Ê¹ÓÃËÄÔªÊı³õÊ¼»¯ÀîÈº
    Sophus::SO3 SO3_Q(Q1);
    cout << "SO(3) SO3_Q from Quaterion" << SO3_Q << endl << endl;

    /****************************************************************************
     3.1 Ê¹ÓÃĞı×ª½Ç£¨Öá½Ç£©µÄ¸÷¸öÔªËØ¶ÔÓ¦µÄ´úÊıÖµÀ´³õÊ¼»¯ÀîÈº

     ×¢Òâ£ºÖ±½ÓÊ¹ÓÃĞı×ª½ÇAngleAxis»òÊÇĞı×ª½Ç¶È¶ÔÓ¦µÄÏòÁ¿(Vector3d=AngleAxis.axis()*AngleAxis.angle())¶ÔÀîÈº½øĞĞ³õÊ¼»¯ÊÇ²»ĞĞµÄ£¬ÒòÎªSO3ÀîÈºÃ»ÓĞ¶ÔÓ¦µÄ¹¹Ôìº¯Êı¡£
    Ò²¼´ÊÇÊ¹ÓÃÏÂÁĞ·½·¨ÊÇ´íÎóµÄ£º

     Sophus::SO3 SO3_A(A1);//Ö±½ÓÊ¹ÓÃĞı×ª½Ç¶ÔÀîÈº³õÊ¼»¯
     Sophus::SO3 SO3_A(A1.axis()*A1.angle());//Ö±½ÓÊ¹ÓÃĞı×ª½Ç¶È¶ÔÓ¦µÄÏòÁ¿(Vector3d=AngleAxis.axis()*AngleAxis.angle())¶ÔÀîÈº½øĞĞ³õÊ¼»¯

     Ö»ÄÜÊ¹ÓÃĞı×ª½Ç¶ÔÓ¦µÄÏòÁ¿µÄÃ¿Ò»¸öÎ¬¶È½øĞĞ¸³Öµ£¬¶ÔÓ¦ÓÚSO3µÄÕâÑùÒ»¸ö¹¹Ôìº¯ÊıSO3(double rot_x, double rot_y, double rot_z);

    *******************************************************************************/

    //3.1.1 Ê¹ÓÃĞı×ª½Ç¶È¶ÔÓ¦µÄÏòÁ¿(Vector3d=AngleAxis.axis()*AngleAxis.angle())ÖĞµÄ¸÷¸öÔªËØ¶ÔÀîÈº½øĞĞ³õÊ¼»¯
    Sophus::SO3 SO3_A1((A1.axis() * A1.angle())(0), (A1.axis() * A1.angle())(1), (A1.axis() * A1.angle())(2));
    cout << "SO(3) SO3_A1 from AngelAxis1" << SO3_A1 << endl << endl;

    //3.1.2 Ê¹ÓÃĞı×ª½Ç¶È¶ÔÓ¦µÄÏòÁ¿(Vector3d=AngleAxis.axis()*AngleAxis.angle())ÖĞµÄ¸÷¸öÔªËØ¶ÔÀîÈº½øĞĞ³õÊ¼»¯
    Sophus::SO3 SO3_A2(M_PI / 2 * 0, M_PI / 2 * 0, M_PI / 2 * 1);
    cout << "SO(3) SO3_A2 from AngleAixs2" << SO3_A2 << endl << endl;

    //3.2 ÓÉÓÚĞı×ª½Ç£¨Öá½Ç£©ÓëÀî´úÊıso(3)¶ÔÓ¦,ËùÒÔÖ±½ÓÊ¹ÓÃĞı×ª½ÇµÄÖµ»ñµÃse(3),½ø¶øÔÙÍ¨¹ıSophus::SO3::exp()»ñµÃ¶ÔÓ¦µÄSO(3)
    Eigen::Vector3d V1(0, 0, M_PI / 2);//so3ÔÚEigenÖĞÓÃVector3d±íÊ¾
    Sophus::SO3 SO3_V1 = Sophus::SO3::exp(V1);
    cout << "SO(3) SO3_V1 from SO3::exp()" << SO3_V1 << endl << endl;


    //¶ş¡¢SO(3)Óëso(3)µÄÏà»¥×ª»»£¬ÒÔ¼°so3¶ÔÓ¦µÄhatºÍvee²Ù×÷

    Eigen::Vector3d so3_V1 = SO3_V1.log();//so(3)ÔÚSophus(Eigen)ÖĞÓÃvector3d±íÊ¾,Ê¹ÓÃ¶ÔÊıÓ³Éä»ñµÃÀîÈº¶ÔÓ¦µÄÀî´úÊı
    cout << "so(3) so3_V1 from SO3_V1" << so3_V1.transpose() << endl << endl;


    Sophus::SO3 SO3_V2 = Sophus::SO3::exp(so3_V1);//Ê¹ÓÃÖ¸ÊıÓ³Éä½«Àî´úÊı×ª»¯ÎªÀîÈº
    cout << "SO(3) so3_V2 from so3_V1" <<SO3_V2 << endl << endl;


    Eigen::Matrix3d M_so3_V1 = Sophus::SO3::hat(so3_V1);//hatÎªÏòÁ¿µ½Æä¶ÔÓ¦µÄ·´¶Ô³Æ¾ØÕó
    cout << "so3 hat=\n" << M_so3_V1 << endl << endl;

    Eigen::Vector3d V_M = Sophus::SO3::vee(M_so3_V1);//veeÎª·´¶Ô³Æ¾ØÕó¶ÔÓ¦µÄÏòÁ¿
    cout << "so3 vee=\n" << V_M << endl << endl;

    //Èı¡¢ÔöÁ¿ÈÅ¶¯Ä£ĞÍ
    Eigen::Vector3d update_so3(1e-4,0,0);//¼ÙÉè¸üĞÂÁ¿ÎªÕâÃ´¶à
    Eigen::Matrix3d update_matrix=Sophus::SO3::exp(update_so3).matrix();//½«ÀîÈº×ª»»ÎªĞı×ª¾ØÕó
    cout<<"SO3 update Matrix=\n"<<update_matrix<<endl<<endl;

    Sophus::SO3 SO3_updated=Sophus::SO3::exp(update_so3)*SO3_R;
    cout<<"SO3 updated = \n"<<SO3_updated<<endl;

    Eigen::Matrix3d SO3_updated_matrix=SO3_updated.matrix();//½«ÀîÈº×ª»»ÎªĞı×ª¾ØÕó
    cout<<"SO3 updated Matrix = \n"<<SO3_updated_matrix<<endl<<endl;


//******************************************************************·Ö¸îÏß***********************************************************************************
    cout<<"************************************ SPLIT *************************************************"<<endl<<endl;

    Eigen::AngleAxisd A2(M_PI/2,Eigen::Vector3d(0,0,1));
    Eigen::Matrix3d R2=A2.matrix();
    Eigen::Quaterniond Q2(A2);
    Sophus::SO3 SO3_2(R2);

    //Ò»¡¢³õÊ¼»¯Àî´úÊıµÄ¼¸ÖÖ·½Ê½
    Eigen::Vector3d t(1,0,0);

    //1. Ê¹ÓÃĞı×ª¾ØÕóºÍÆ½ÒÆÏòÁ¿À´³õÊ¼»¯SE3
    Sophus::SE3 SE_Rt(R2,t);
    cout<<"SE3 SE_Rt from  Rotation_Matrix and Transform=\n"<<SE_Rt<<endl<<endl;//×¢Òâ¾¡¹ÜSE(3)ÊÇ¶ÔÓ¦Ò»¸ö4*4µÄ¾ØÕó,µ«ÊÇÊä³öSE(3)Ê±æ˜????Ò»¸öÁùÎ¬ÏòÁ¿Êä³öµÄ,ÆäÖĞÇ°Ç°ÈıÎ»Îª¶ÔÓ¦µÄso3,ºó3Î¬¶ÈÎªÊµé??µÄÆ½ÒÆÁ¿t£¬¶ø²»ÊÇse3ÖĞµÄÆ½ÒÆ·ÖÁ¿
    //2. Ê¹ÓÃËÄÔªÊıºÍÆ½ÒÆå??Á¿À´³õÊ¼»¯SE3
    Sophus::SE3 SE_Qt(Q2,t);
    cout<<"SE3 SE_Qt from  Quaterion and Transform=\n"<<SE_Qt<<endl<<endl;
    //3. Ê¹ÓÃSO3ºÍÆ½ÒÆÏòÁ¿À´³õÊ¼»¯SE3
    Sophus::SE3 SE_St(SO3_2,t);
    cout<<"SE3 SE_St from  SO3 and Transform=\n"<<SE_St<<endl<<endl;

    //¶ş¡¢SE(3)Óëse(3)µÄÏà»¥×ª»»£¬ÒÔ¼°se3¶ÔÓ¦µÄhatå??vee²Ù×÷
    Sophus::Vector6d se3_Rt=SE_Rt.log();//se(3)ÔÚSophusÖĞÓÃVector6d±íÊ¾,Ê¹ÓÃ¶ÔÊıÓ³Éä»ñµÃÀîÈº¶ÔÓ¦µÄÀî´úÊı
    cout<<"se(3) se3_Rt from SE3_Rt\n"<<se3_Rt<<endl<<endl;//se3Êä³öµÄÊÇä¸????ÁùÎ¬¶ÈÏòÁ¿,ÆäÖĞÇ°3Î¬ÊÇÆ½ÒÆ·ÖÁ¿,ºó3Î¬¶ÈÊÇĞı×ª·ÖÁ¿

    Sophus::SE3 SE3_Rt2=Sophus::SE3::exp(se3_Rt);//Ê¹ÓÃÖ¸ÊıÓ³Éä½«Àî´úÊıè??»¯ÎªÀîÈº
    cout<<"SE(3) SO3_Rt2 from se3_Rt"<<SE3_Rt2<<endl<<endl;

    Sophus::Matrix4d M_se3_Rt=Sophus::SE3::hat(se3_Rt);
    cout<<"se(3) hat=\n"<<M_se3_Rt<<endl<<endl;

    Sophus::Vector6d V_M_se3=Sophus::SE3::vee(M_se3_Rt);
    cout<<"se(3) vee=\n"<<V_M_se3<<endl<<endl;




    //Èı¡¢ÔöÁ¿ÈÅ¶¯Ä£ĞÍ

    Sophus::Vector6d update_se3=Sophus::Vector6d::Zero();
    update_se3(0)=1e-4d;

    cout<<"update_se3\n"<<update_se3.transpose()<<endl<<endl;

    Eigen::Matrix4d update_matrix2=Sophus::SE3::exp(update_se3).matrix();//½«ÀîÈº×ª»»ÎªĞı×ª¾ØÕó
    cout<<"update matrix=\n"<<update_matrix2<<endl<<endl;

    Sophus::SE3 SE3_updated=Sophus::SE3::exp(update_se3)*SE3_Rt2;
    cout<<"SE3 updated=\n"<<SE3_updated<<endl<<endl;

    Eigen::Matrix4d SE3_updated_matrix=SE3_updated.matrix();//½«ÀîÈº×ª»»ÎªĞı×ª¾ØÕó
    cout<<"SE3 updated Matrix=\n"<<SE3_updated_matrix<<endl<<endl;

    return 0;

}
