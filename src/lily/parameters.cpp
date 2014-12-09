#include <QtXml>

#include "parameters.h"

Parameters::Parameters()
    :start_frame_(-1), end_frame_(-1), key_frame_(-1)
{}

Parameters::~Parameters()
{}

bool Parameters::load(const std::string& filename)
{
    QFile file(filename.c_str());
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return false;
    }
    QDomDocument doc("blooming_flowers");
    if (!doc.setContent(&file)) {
        file.close();
        return false;
    }
    QDomElement root = doc.documentElement();


    // frame sequence information
    QDomElement sequenceElement = root.firstChildElement(QString("chosen_frame_sequence"));
    start_frame_ = sequenceElement.attribute("start_frame").toInt();
    end_frame_ = sequenceElement.attribute("end_frame").toInt();
    key_frame_ = sequenceElement.attribute("key_frame").toInt();

    file.close();
    return true;

}

bool Parameters::save(const std::string& filename)
{
    QFile file(filename.c_str());
    if (!file.open(QIODevice::WriteOnly)) {
        return false;
    }

    QDomDocument doc("blooming_flowers");
    QDomProcessingInstruction xml_declaration = doc.createProcessingInstruction("xml", "version=\"1.0\"");
    doc.appendChild(xml_declaration);

    QDomElement rootElement = doc.createElement(QString("blooming_flowers"));
    doc.appendChild(rootElement);

    // frame sequence information
    QDomElement sequenceElement = doc.createElement(QString("chosen_frame_sequence"));
    sequenceElement.setAttribute("start_frame", start_frame_);
    sequenceElement.setAttribute("end_frame", end_frame_);
    sequenceElement.setAttribute("key_frame", key_frame_);
    rootElement.appendChild(sequenceElement);

    QTextStream text_stream(&file);
    text_stream << doc.toString();
    file.close();

    return true;

}