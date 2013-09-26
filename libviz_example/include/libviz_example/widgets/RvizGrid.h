/*
 * RvizGrid.h
 *
 *  Created on: Apr 16, 2013
 *      Author: ros developer 
 */

#ifndef RVIZGRID_H_
#define RVIZGRID_H_

#include <QWidget>
#include <boost/shared_ptr.hpp>
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include <rviz/default_plugin/grid_display.h>

#define DECLARE_BOOST_PTR_TYPE(type,type_prefix) typedef boost::shared_ptr<type> type_prefix ## Ptr

namespace libviz_example { namespace widgets
{

DECLARE_BOOST_PTR_TYPE(rviz::VisualizationManager,VisualizationManager);
DECLARE_BOOST_PTR_TYPE(rviz::RenderPanel,RenderPanel);
DECLARE_BOOST_PTR_TYPE(rviz::GridDisplay,GridDisplay);
DECLARE_BOOST_PTR_TYPE(rviz::Display,Display);

class RvizGrid: public QWidget
{
public:
	typedef boost::shared_ptr<RvizGrid> Ptr;

Q_OBJECT
public:
	RvizGrid(QWidget* parent = 0);
	virtual ~RvizGrid();

	void show();

protected:
	bool setup();

private Q_SLOTS:
	void set_thickness(int percent);
	void set_cell_size(int percent);
	void set_alpha(double val);

private:
	VisualizationManagerPtr manager_ptr_;
	RenderPanelPtr render_panel_ptr_;
	GridDisplayPtr grid_display_ptr_;

};

}	} /* namespace libviz_example */
#endif /* RVIZGRID_H_ */
