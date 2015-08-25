
#include "flower.h"
#include "main_window.h"
#include "registrator.h"
#include "scene_widget.h"

Flower::Flower()
{
}

// deep copy
Flower::Flower(const Flower& flower)
{
    for (size_t i = 0, i_end = flower.getPetals().size(); i < i_end; ++ i){
        petals_.push_back(flower.getPetals()[i]);
        petal_order_.push_back(flower.getPetalOrder()[i]);
    }
}

Flower::~Flower()
{
}

void Flower::reorder()
{
    std::sort(petals_.begin(), petals_.end(), [&](Petal& p1, Petal& p2){
        std::string p1_name = p1.getObjName();
        std::string p2_name = p2.getObjName();
        if (p1_name < p2_name)
            return true;
        else return false;
    });

    // manually
    petal_order_.resize(petals_.size());

    petal_order_[0] = 0;
    petal_order_[1] = 0;
    petal_order_[2] = 0;
    petal_order_[3] = 1;
    petal_order_[4] = 1;
    petal_order_[5] = 1;

}

void Flower::save(const std::string& flower_path)
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
    {
        Petal& petal = petals_[i];
        std::string& petal_file = flower_path + QString("/%1").arg(QString(petal.getObjName().c_str())).toStdString();
        
        petal.save(petal_file, true);
    }
}

void Flower::save(const std::string& flower_folder, int frame)
{
    QDir flower_frame(flower_folder.c_str());
    QString frame_file = QString("frame_%1").arg(frame, 5, 10, QChar('0'));
    flower_frame.mkdir(frame_file);
    QString flower_path = flower_frame.absolutePath() + "/" + frame_file;
    
    save(flower_path.toStdString());
}

void Flower::load(const std::string& flower_path)
{
    QDir flowers_dir = QDir(flower_path.c_str());
    QStringList allowed_file_extensions;
    allowed_file_extensions.push_back("*.obj");
    flowers_dir.setNameFilters(allowed_file_extensions);

    QStringList points_entries = flowers_dir.entryList();

    for (size_t i = 0, i_end = points_entries.size(); i < i_end; ++ i)
    {
        std::string& petal_file = flower_path + "/" + points_entries.at(i).toStdString();
        Petal petal;
        petal.load(petal_file);
        petals_.push_back(petal);
    }
}

void Flower::clear()
{
    petals_.clear();
}

void Flower::show()
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
    {
        Petal& petal = petals_[i];
        petal.getColorId() = i;
        MainWindow::getInstance()->getSceneWidget()->addSceneChild(&petal);

        if (!petal.getSkeleton()->isEmpty())
        {
            petal.getSkeleton()->show(); // show related skeleton
            //petal.getSkeleton()->setHiddenState(true);
        }
    }
}


void Flower::update()
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
    {
        Petal& petal = petals_[i];
        petal.expire();

        petal.getSkeleton()->expire();
    }
}

void Flower::hide()
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
    {
        Petal& petal = petals_[i];
        //MainWindow::getInstance()->getSceneWidget()->removeSceneChild(&petal);  // will cause error
        petal.setHiddenState(true);
        petal.expire();

        petal.getSkeleton()->setHiddenState(true);
        petal.getSkeleton()->expire();
    }

}

// since removeSceneChild(&petal) will cause error(I don't know why...), here in order to remove the flower
// in the graph scene, I decide to remove all...
void Flower::remove()
{
    //for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
    //{
    //    Petal& petal = petals_[i];
    //    MainWindow::getInstance()->getSceneWidget()->removeSceneChild(&petal);  // will cause error
    //}

    MainWindow::getInstance()->getSceneWidget()->removeSceneChildren();
}

void Flower::rotate(const osg::Matrix& rot_matrix)
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
        petals_[i].rotate(rot_matrix);
    return;
}

int Flower::contains(Petal* petal)
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
    {
        if (&petals_[i] == petal)
            return i;
    }
    return -1;
}

void Flower::determineWeights(PointCloud* aligned_cloud)
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
    {
        petals_[i].determineWeights(aligned_cloud, i);
    }
}

void Flower::determineVisibility()
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
        petals_[i].initializeVisibility();

    // default, six views
    const int view_num = 6;
    double angle = (2 * M_PI) / view_num;

    Registrator* registrator = MainWindow::getInstance()->getRegistrator();
    osg::Matrix rotation = registrator->getRotationMatrix(angle);

    for (size_t i = 0; i < view_num; ++ i)
    {
        rotate(rotation);
        determineIntersection();
        update();
    }
}

// an estimation method of visibility for our special case by geometry relationship, not by accurate algorithm
void Flower::determineIntersection()
{
    osg::BoundingSphere bounding_sphere = MainWindow::getInstance()->getSceneWidget()->getBoundingSphere();
    
    osg::Vec3 plane_normal = osg::Vec3(0.0, 0.0, 1.0) ^ bounding_sphere.center();
    plane_normal.normalize();

    float cos_alpha = osg::Vec3(0.0, 0.0, 1.0) * bounding_sphere.center() / bounding_sphere.center().length();
    float alpha = acos(cos_alpha);

    osg::Matrix rot_matrix = osg::Matrix::identity();
    rot_matrix = rot_matrix * osg::Matrix::translate(-bounding_sphere.center());
    rot_matrix = rot_matrix * osg::Matrix::rotate(alpha, plane_normal);
    rot_matrix = rot_matrix * osg::Matrix::translate(bounding_sphere.center());

    osg::ref_ptr<osg::Group> scene_root = MainWindow::getInstance()->getSceneWidget()->getSceneRoot();

    osg::Vec3d z_start = bounding_sphere.center() - osg::Vec3d(0.0, 0.0, bounding_sphere.radius());
    osg::Vec3d z_end = bounding_sphere.center() + osg::Vec3d(0.0, 0.0, bounding_sphere.radius());

    osg::Vec3d deltaRow(bounding_sphere.radius()*0.02, 0.0, 0.0);
    osg::Vec3d deltaColumn(0.0, bounding_sphere.radius()*0.02, 0.0);

    const int numRows = 50;
    const int numColumns = 50;

    osg::ref_ptr<osgUtil::IntersectorGroup> intersectorGroup = new osgUtil::IntersectorGroup();

    for(int r = -numRows; r <= numRows; ++r)
    {
        for(int c = -numColumns; c <= numColumns; ++c)
        {
            osg::Vec3d s = z_start + deltaColumn * double(c) + deltaRow * double(r);
            osg::Vec3d e = z_end + deltaColumn * double(c) + deltaRow * double(r);
            s = rot_matrix.preMult(s);
            e = rot_matrix.preMult(e);
            osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector(s, e);
            intersectorGroup->addIntersector( intersector.get() );
        }
    }

    osgUtil::IntersectionVisitor intersectVisitor( intersectorGroup.get());
    scene_root->accept(intersectVisitor);

    if (intersectorGroup->containsIntersections())
    {
        std::cout<<"Found intersections "<<std::endl;
        osgUtil::IntersectorGroup::Intersectors& intersectors = intersectorGroup->getIntersectors();
        for(osgUtil::IntersectorGroup::Intersectors::iterator intersector_itr = intersectors.begin();
            intersector_itr != intersectors.end(); ++intersector_itr)
        {
            osgUtil::LineSegmentIntersector* lsi = dynamic_cast<osgUtil::LineSegmentIntersector*>(intersector_itr->get());
            if (lsi)
            {
                osgUtil::LineSegmentIntersector::Intersection& intersection = lsi->getFirstIntersection();
                osg::NodePath& node_path = intersection.nodePath;
                osg::ref_ptr<Petal> intersection_object = NULL;

                while (!node_path.empty()) {
                    intersection_object = dynamic_cast<Petal*>(node_path.back());
                    if (intersection_object != NULL) {
                        break;
                    }
                    node_path.pop_back();
                }

                if (intersection_object != NULL)
                {
                    osg::Vec3d intersect_point = intersection.getWorldIntersectPoint();
                    float min_dist = std::numeric_limits<float>::max();
                    int min_idx = 0;
                    for (size_t k = 0, k_end = intersection.indexList.size(); k < k_end; ++ k)
                    {
                        osg::Vec3 closest_point = intersection_object->getVertices()->at(intersection.indexList[k]);
                        float dist = (intersect_point - closest_point).length();
                        if (dist < min_dist)
                        {
                            min_dist = dist;
                            min_idx = intersection.indexList[k];
                        }
                    }
                    intersection_object->getVisibility().at(min_idx) = 1;
                }

            }
        }
    }
    else
    {
        std::cout << "No intersections" << std::endl;
    }
}

void Flower::determineVisibility(PointCloud* aligned_cloud)
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
    {
        petals_[i].determineVisibility(aligned_cloud, i);
    }
}

void Flower::initVisibility()
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
        petals_[i].initializeVisibility();
}

void Flower::setTextureState(bool is_shown)
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
        petals_[i].getShowTexture() = is_shown;
}

void Flower::setSkeletonState(bool is_shown)
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
        petals_[i].showSkeletonState(is_shown);
}

// all petals have the same state
bool Flower::getTextureState()
{
    if (petals_.empty()) return false;
    return petals_[0].getShowTexture();
}

bool Flower::getSkeletonState()
{
    if (petals_.empty()) return false;
    return petals_[0].getShowSkeleton();
}

void Flower::setTrajsIndices(Flower* ref_flower)
{
    Petals& ref_petals = ref_flower->getPetals();
    for (size_t i = 0, i_end = ref_petals.size(); i < i_end; ++ i)
    {
        petals_[i].getTrajsVertices() = ref_petals[i].getTrajsVertices();
    }
}






FlowersViewer::FlowersViewer(const std::string& flowers_folder)
    :flowers_folder_(flowers_folder),
    current_frame_(-1),
    start_frame_(-1),
    end_frame_(-1),
    texture_state_(false),
    skeleton_state_(false)
{

}

FlowersViewer::~FlowersViewer()
{

}

void FlowersViewer::setFrame(int frame)
{
    current_frame_ = frame;
}

void FlowersViewer::getFlower(int frame)
{
    // store showing texture and skeleton flag
    texture_state_ = current_flower_.getTextureState();
    skeleton_state_ = current_flower_.getSkeletonState();

    current_frame_ = frame;
    Flower current_flower;

    std::string frame_file = QString("frame_%1").arg(current_frame_, 5, 10, QChar('0')).toStdString();
    std::string flower_path = flowers_folder_ + "/" + frame_file;

    current_flower.load(flower_path);
    current_flower_ = current_flower;

    current_flower_.setTextureState(texture_state_);
    current_flower_.setSkeletonState(skeleton_state_);
}

void FlowersViewer::getFlower()
{
    getFlower(start_frame_);
}

void FlowersViewer::next()
{
    current_frame_ = current_frame_ + 1;
    if (current_frame_ > end_frame_)
        current_frame_ = end_frame_;

    getFlower(current_frame_);
}

void FlowersViewer::previous()
{
    current_frame_ = current_frame_ - 1;
    if (current_frame_ < start_frame_)
        current_frame_ = start_frame_;

    getFlower(current_frame_);
}

void FlowersViewer::show()
{
    current_flower_.show();
}

void FlowersViewer::update()
{
    current_flower_.update();
}

void FlowersViewer::computeFrameRange()
{
    start_frame_ = end_frame_ = -1;

    QString root_path = QString(flowers_folder_.c_str());

    if (root_path.contains("frame_")) {
        start_frame_ = end_frame_ = root_path.right(4).toInt();
        return;
    }

    if (root_path.contains("flowers"))
    {
        QStringList points_entries = QDir(root_path).entryList();
        extractStartEndFrame(points_entries, start_frame_, end_frame_);
        return;
    }

    current_frame_ = start_frame_;

    return;

}

void FlowersViewer::extractStartEndFrame(const QStringList& entries, int& start_frame, int& end_frame)
{
    start_frame = std::numeric_limits<int>::max();
    end_frame = std::numeric_limits<int>::min();

    for (QStringList::const_iterator entries_it = entries.begin();
        entries_it != entries.end(); ++ entries_it)
    {
        if (!entries_it->contains("frame_"))
            continue;

        int index = entries_it->right(4).toInt();
        if (start_frame > index)
            start_frame = index;
        if (end_frame < index)
            end_frame = index;
    }

    return;
}

osg::ref_ptr<Flower> FlowersViewer::flower(int frame)
{
    std::string frame_file = QString("frame_%1").arg(frame, 5, 10, QChar('0')).toStdString();
    std::string flower_path = flowers_folder_ + "/" + frame_file;

    osg::ref_ptr<Flower> flr = new Flower;
    flr->load(flower_path);
    
    return flr;
}