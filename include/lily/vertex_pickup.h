#pragma once

#include <osgGA/GUIEventHandler>
#include <osgViewer/ViewerEventHandlers>
#include <string>

//////////////////////////////////////////////////////////////////////////
// created 2013-10-01
//////////////////////////////////////////////////////////////////////////
class PickupVertexHandler : public osgGA::GUIEventHandler
{
public:
	PickupVertexHandler( osg::Group* root );
	
	//************************************
	// Method:    handle
	// Returns:   bool
	// Function:  
	// Time:      2014/04/09 
	// Author:    qian
	//************************************
	bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa );


	virtual void pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea);

protected:
	void createText();
	void createGeo();


	void setPointAttributes( osg::Geometry* selector );
	bool setPointPosition( osg::Geometry* selector, osg::Vec3 pos);

protected:
	osg::ref_ptr<osg::Group>       m_pickupRoot;
	osg::ref_ptr<osgText::Text>    m_updateText;
	osg::Vec4                      m_selectedColor;
	osg::ref_ptr<osg::Geometry>   m_selectedVertexGeo;

};
