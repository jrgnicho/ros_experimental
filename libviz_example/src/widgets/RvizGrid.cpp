/*
 * RvizGrid.cpp
 *
 *  Created on: Apr 16, 2013
 *      Author: ros developer 
 */

#include <libviz_example/widgets/RvizGrid.h>

#include <QSlider>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QVBoxLayout>
#include <rviz/ogre_helpers/grid.h>

namespace libviz_example {	namespace widgets{

//using namespace libviz_example::widgets;

RvizGrid::RvizGrid(QWidget *parent)
	:QWidget(parent)
{
	// TODO Auto-generated constructor stub

}

RvizGrid::~RvizGrid()
{

}

void RvizGrid::show()
{
	setup();
	QWidget::show();
}

void RvizGrid::set_cell_size(int percent)
{
	if(grid_display_ptr_)
	{
		grid_display_ptr_->subProp("Cell Size")->setValue(percent/10.f);
	}
}

void RvizGrid::set_thickness(int percent)
{
	if(grid_display_ptr_)
	{
		grid_display_ptr_->subProp("Line Style")->subProp("Line Width")->setValue(percent/100.f);
	}

}

void RvizGrid::set_alpha(double val)
{
	if(grid_display_ptr_)
	{
		grid_display_ptr_->subProp("Alpha")->setValue(val);
	}
}

bool RvizGrid::setup()
{
	// thickness label setup
	QLabel* thickness_label = new QLabel(" Line Thickness");

	// thickness slider setup
	QSlider* thickness_slider = new QSlider(Qt::Horizontal);
	thickness_slider->setMinimum(1);
	thickness_slider->setMaximum(100);

	// cell size label setup
	QLabel* cell_size_label = new QLabel(" Cell Size ");

	// cell size slider setup
	QSlider *cell_size_slider = new QSlider(Qt::Horizontal);
	cell_size_slider->setMinimum(1);
	cell_size_slider->setMaximum(100);

	// alpha label setup
	QLabel* alpha_label = new QLabel(" Alpha ");

	// alpha spinner setup
	QDoubleSpinBox* alpha_spin_box = new QDoubleSpinBox();
	alpha_spin_box->setMinimum(0.0f);
	alpha_spin_box->setMaximum(1.0f);
	alpha_spin_box->setSingleStep(0.1f);

	// control layout setup
	QGridLayout* grid_control_layout = new QGridLayout();
	grid_control_layout->addWidget(thickness_label,0,0);
	grid_control_layout->addWidget(thickness_slider,0,1);
	grid_control_layout->addWidget(cell_size_label,1,0);
	grid_control_layout->addWidget(cell_size_slider,1,1);
	grid_control_layout->addWidget(alpha_label,2,0);
	grid_control_layout->addWidget(alpha_spin_box,2,1);

	// main layout setup
	QVBoxLayout* main_layout = new QVBoxLayout();
	render_panel_ptr_ = RenderPanelPtr(new rviz::RenderPanel());
	main_layout->addLayout(grid_control_layout);
	main_layout->addWidget(render_panel_ptr_.get());
	setLayout(main_layout);

	// signals/slots connection setup
	connect(thickness_slider, SIGNAL(valueChanged(int)) , this , SLOT(set_thickness(int)) );
	connect(cell_size_slider, SIGNAL(valueChanged(int)) , this , SLOT(set_cell_size(int)) );
	connect(alpha_spin_box, SIGNAL(valueChanged(double)) , this , SLOT(set_alpha(double)) );

	// initialize top level rviz classes
	manager_ptr_ = VisualizationManagerPtr(new rviz::VisualizationManager(render_panel_ptr_.get()));
	render_panel_ptr_->initialize(manager_ptr_->getSceneManager(),manager_ptr_.get());
	manager_ptr_->initialize();
	manager_ptr_->startUpdate();

	// rviz grid display setup
	grid_display_ptr_ = GridDisplayPtr(
			dynamic_cast<rviz::GridDisplay*>(manager_ptr_->createDisplay("rviz/Grid","adjustable grid",true)));
	grid_display_ptr_->subProp("Line Style")->setValue("Billboards");
	grid_display_ptr_->subProp("Color")->setValue(Qt::yellow);

	thickness_slider->setValue(25);
	cell_size_slider->setValue(10);
	alpha_spin_box->setValue(0.4f);

	return true;
}

} }/* namespace libviz_example */
