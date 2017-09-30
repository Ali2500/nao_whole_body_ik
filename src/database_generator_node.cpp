#include <nao_whole_body_ik/database_generator.h>
#include <nao_whole_body_ik/database_reader.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "database_generator");
	ros::NodeHandle node_handle;

  nao_whole_body_ik::DatabaseGenerator db_gen;
	db_gen.generateMultiFootDatabase();

	ros::shutdown();
	return 0;
}
