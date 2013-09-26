/*
 * rviz_adjustable_grid_node.cpp
 *
 *  Created on: Apr 17, 2013
 *      Author: ros developer 
 */

#include <libviz_example/widgets/RvizGrid.h>
#include <QApplication>
#include <ros/ros.h>

int main(int argc,char** argv)
{
	using namespace libviz_example::widgets;

	if( !ros::isInitialized() )
	{
		ros::init( argc, argv,"rviz_adjustable_grid");
	}
	QApplication app(argc,argv);

	RvizGrid::Ptr rviz_grid(new RvizGrid());
	rviz_grid->show();
	app.exec();

}
