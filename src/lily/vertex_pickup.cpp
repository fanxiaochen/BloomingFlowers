#include "vertex_pickup.h"
#include <osg/Point>
#include "point_intersector.h"
#include <sstream>



PickupVertexHandler::PickupVertexHandler( osg::Group* root )
{
	m_pickupRoot = new osg::Group;
	root->addChild( m_pickupRoot );
	m_selectedColor = osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f);

	createText();
	createGeo();
}





void PickupVertexHandler::setPointAttributes( osg::Geometry* selector )
{
	const osg::Vec4 selectedColor(1.0f, 0.0f, 0.0f, 1.0f);
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array(1);
	(*colors)[0] = selectedColor;

	selector->setDataVariance( osg::Object::DYNAMIC );
	selector->setUseDisplayList( false );
	selector->setUseVertexBufferObjects( true );
	selector->setVertexArray( new osg::Vec3Array(1) );
	selector->setColorArray( colors.get() );
	selector->setColorBinding( osg::Geometry::BIND_OVERALL );
	selector->addPrimitiveSet( new osg::DrawArrays(GL_POINTS, 0, 1) );
}

void PickupVertexHandler::createText()
{
	std::string timesFont("fonts/times.ttf");
	m_updateText = new osgText::Text;
	m_updateText->setCharacterSize(20.0f);
	m_updateText->setColor( m_selectedColor );
	m_updateText->setFont(timesFont);
	m_updateText->setText("");
	m_updateText->setPosition(osg::Vec3(0,0,0));
	m_updateText->setDataVariance(osg::Object::DYNAMIC);
	m_updateText->setAlignment(osgText::Text::LEFT_TOP);
	m_updateText->setPosition( osg::Vec3(150.0f,750.0f,0.0f) );


	// create hdu camera
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable( m_updateText.get() );
	osg::Camera* camera = new osg::Camera;
	camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	camera->setProjectionMatrixAsOrtho2D(0,1280,0,1024);
	camera->setViewMatrix(osg::Matrix::identity());
	camera->setClearMask(GL_DEPTH_BUFFER_BIT);
	camera->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
	camera->getOrCreateStateSet()->setMode(GL_LIGHT_MODEL_TWO_SIDE, osg::StateAttribute::ON);
	camera->addChild( geode );



	m_pickupRoot->addChild(camera);
}

void PickupVertexHandler::createGeo()
{
	m_selectedVertexGeo = new osg::Geometry;
	setPointAttributes( m_selectedVertexGeo.get() );

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable( m_selectedVertexGeo.get() );
	geode->getOrCreateStateSet()->setAttributeAndModes( new osg::Point(10.0f) );
	geode->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

	m_pickupRoot->addChild( geode );
}



bool PickupVertexHandler::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
	switch(ea.getEventType())
	{
	case(osgGA::GUIEventAdapter::PUSH):
		{
			osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
			if (view) pick(view,ea);
			return false;
		}    
	case(osgGA::GUIEventAdapter::KEYDOWN):
		{
			if (ea.getKey()=='v')
			{        
			}
			return false;
		}    
	default:
		return false;
	}
}


void PickupVertexHandler::pick( osgViewer::View* view, const osgGA::GUIEventAdapter& ea )
{
	osg::ref_ptr<PointIntersector> intersector =
		new PointIntersector(osgUtil::Intersector::WINDOW, ea.getX(), ea.getY());
	osgUtil::IntersectionVisitor iv( intersector.get() );
	view->getCamera()->accept( iv );

	if( intersector->containsIntersections() == false)
	{
		m_updateText->setText("");
		m_updateText->dirtyBound();
	}
	else
	{
		PointIntersector::Intersections::iterator hitr = intersector->getIntersections().begin();
		osg::Vec3 pos = hitr->getWorldIntersectPoint();
		setPointPosition( m_selectedVertexGeo.get(), pos);

		std::ostringstream os;
		os << "Vertex " << hitr->primitiveIndex << std::endl;
		m_updateText->setText(os.str());
		m_updateText->dirtyBound();
	}
}

bool PickupVertexHandler::setPointPosition( osg::Geometry* selector, osg::Vec3 pos )
{
	osg::Vec3Array* selVertices = 0;
	selVertices = dynamic_cast<osg::Vec3Array*>(selector->getVertexArray() );
	if( selVertices == 0 || selVertices->size() ==0 )
		return false;
	selVertices->front() = pos;
	selVertices->dirty();
	selector->dirtyBound();
	return true;
}
