/*
	Aseba - an event-based framework for distributed robot control
	Copyright (C) 2007--2016:
		Stephane Magnenat <stephane at magnenat dot net>
		(http://stephane.magnenat.net)
		and other contributors, see authors.txt for details

	This example is based on a first work of Olivier Marti (2010 - 2011).
	Stripped down & cleaned version by Florian Vaussard (2013).
	
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published
	by the Free Software Foundation, version 3 of the License.
	
	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Lesser General Public License for more details.
	
	You should have received a copy of the GNU Lesser General Public License
	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include "dashelinterface.h"

#include <QMessageBox>
#include <QDebug>
#include <dashel/dashel.h>
#include <string>
#include <aseba/common/msg/descriptions-manager.h>
#include <iterator>
#include <cstdlib>
#include <iostream>

#include <libxml/parser.h>
#include <libxml/tree.h>

//#include "dashelinterface.moc"

using namespace std;
using namespace Dashel;
using namespace Aseba;

DashelInterface::DashelInterface() :
	isRunning(false), isConnected(false)
{
}

DashelInterface::~DashelInterface()
{
}

// UTF8 to wstring
static std::wstring widen(const char *src)
{
    const size_t destSize(mbstowcs(0, src, 0)+1);
    std::vector<wchar_t> buffer(destSize, 0);
    mbstowcs(&buffer[0], src, destSize);
    return std::wstring(buffer.begin(), buffer.end() - 1);
}
static std::wstring widen(const std::string& src)
{
    return widen(src.c_str());
}

std::string narrow(const wchar_t* src)
{
    const size_t destSize(wcstombs(0, src, 0)+1);
    std::vector<char> buffer(destSize, 0);
    wcstombs(&buffer[0], src, destSize);
    return std::string(buffer.begin(), buffer.end() - 1);
}
std::string narrow(const std::wstring& src)
{
    return narrow(src.c_str());
}

// Connect to any kind of valid Dashel target (TCP, serial, CAN,...)
void DashelInterface::connectAseba(const QString& dashelTarget)
{
	dashelParams = dashelTarget;
	isRunning = true;
	start();
}

// Connect through a TCP socket
void DashelInterface::connectAseba(const QString& ip, const QString& port)
{
	connectAseba("tcp:" + ip + ";port=" + port);
}

// Cleanly disconnect
void DashelInterface::disconnectAseba()
{
	isRunning = false;
	Dashel::Hub::stop();
	wait();
}

int DashelInterface::loadAsebaScript(const std::string & AsebaScript)
{
    xmlDoc *doc = xmlReadFile(AsebaScript.c_str(), NULL, 0);
    if (!doc)
    {
        std::cout<<"cannot read/find the file"<<AsebaScript<<std::endl;
        return FALSE;
    }
    if(!stream)
    {
        std::cout<<"Target not detected, no script can be loaded"<<std::endl;
        return FALSE;
    }
    xmlNode *domRoot = xmlDocGetRootElement(doc);

    commonDefinitions.events.clear();
    commonDefinitions.constants.clear();
    allVariables.clear();

    int noNodeCount = 0;
    bool wasError = false;
    if (!xmlStrEqual(domRoot->name, BAD_CAST("network")))
    {
        std::cout<<"root node is not \"network\", XML considered as invalid"<<std::endl;
        return FALSE;
    }
    else for (xmlNode *domNode = xmlFirstElementChild(domRoot); domNode; domNode = domNode->next)
    {
        if (domNode->type == XML_ELEMENT_NODE)
        {
            if (xmlStrEqual(domNode->name, BAD_CAST("node")))
            {
                xmlChar *name = xmlGetProp(domNode, BAD_CAST("name"));
                if (!name)
                    std::cout<<"missing \"name\" attribute in \"node\" entry"<<std::endl;
                else
                {
                    const string _name((const char *)name);
                    xmlChar * text = xmlNodeGetContent(domNode);
                    if (!text)
                        std::cout<<"missing text in \"node\" entry"<<std::endl;
                    else
                    {
                        unsigned preferedId(0);
                        xmlChar *storedId = xmlGetProp(domNode, BAD_CAST("nodeId"));
                        if (storedId)
                        {
                            preferedId = unsigned(atoi((char*)storedId));
                        }
                        bool ok;

                        //get description of the node in order (Mandatory!!!)
                        GetDescription getDes;
                        getDes.serialize(stream);
                        stream->flush();

                        sleep(1); //Need to wait 1 sec to receive the description of the node

                        //Call Description Manager for the Target description
                        unsigned nodeId(getNodeId(widen(_name), preferedId, &ok));
                        //std::cout<<"Found the name: "<<_name<<" and ID: "<<storedId<<std::endl;
                        if (ok)//ok
                        {
                            std::wistringstream is(widen((const char *)text));
                            Error error;
                            BytecodeVector bytecode;
                            unsigned allocatedVariablesCount;

                            //Instance compiler to load the code using description manager
                            // -> common/msg/descriptions-manager.h/.cpp
                            Compiler compiler;
                            compiler.setTargetDescription(getDescription(nodeId));
                            compiler.setCommonDefinitions(&commonDefinitions);
                            bool result = compiler.compile(is, bytecode, allocatedVariablesCount, error);

                            if (result)
                            {
                                sendBytecode(stream, nodeId, std::vector<uint16>(bytecode.begin(), bytecode.end()));
                                Run msg(nodeId); //Run the Aseba script
                                msg.serialize(stream);
                                stream->flush();
                                std::cout<<"The Script \""<<AsebaScript<<"\" has been loaded into node "<<_name<<". Enjoy!"<<std::endl;
                            }
                            else
                            {
                                std::cout<<"compilation failed"<<std::endl;
                                return FALSE;
                            }
                        }

                        else
                            noNodeCount++
                                    ;
                        xmlFree(text);
                    }
                    xmlFree(name);
                }
            }
            else if (xmlStrEqual(domNode->name, BAD_CAST("event")))
            {
                // get attributes
                xmlChar *name = xmlGetProp(domNode, BAD_CAST("name"));
                if (!name)
                    std::cout<<"missing event"<<std::endl;
                xmlChar *size = xmlGetProp(domNode, BAD_CAST("size"));
                if (!size)
                    std::cout<<"missing size"<<std::endl;
                // add event
                if (name && size)
                {
                    int eventSize(atoi((const char *)size));
                    if (eventSize > ASEBA_MAX_EVENT_ARG_SIZE)
                    {
                        std::cout<<"event too big"<<std::endl;
                        break;
                    }
                    else
                    {
                        commonDefinitions.events.push_back(NamedValue(widen((const char *)name), eventSize));
                    }
                }
                // free attributes
                if (name)
                    xmlFree(name);
                if (size)
                    xmlFree(size);
            }
            else if (xmlStrEqual(domNode->name, BAD_CAST("constant")))
            {
                // get attributes
                xmlChar *name = xmlGetProp(domNode, BAD_CAST("name"));
                if (!name)
                    std::cout<<"missing event"<<std::endl;
                xmlChar *value = xmlGetProp(domNode, BAD_CAST("value"));
                if (!value)
                    std::cout<<"missing constant"<<std::endl;
                // add constant if attributes are valid
                if (name && value)
                {
                    commonDefinitions.constants.push_back(NamedValue(widen((const char *)name), atoi((const char *)value)));
                }
                // free attributes
                if (name)
                    xmlFree(name);
                if (value)
                    xmlFree(value);
            }
            else
                std::cout<<"Unknown XML node seen in .aesl file"<<std::endl;

        }

    }
    xmlFreeDoc(doc);
}
// Message coming from a node.
// Consider _only_ UserMessage. Discard other types of messages (debug, etc.)
void DashelInterface::incomingData(Dashel::Stream *stream)
{
	Aseba::Message *message = Aseba::Message::receive(stream);
	Aseba::UserMessage *userMessage = dynamic_cast<Aseba::UserMessage *>(message);

    //A description manager for loading the Aseba Scripts
    DescriptionsManager::processMessage(message);

	if (userMessage)
		emit messageAvailable(userMessage);
	else
		delete message;
}

// Send a UserMessage with ID 'id', and optionnally some data values
void DashelInterface::sendEvent(unsigned id, const QVector<int>& values)
{
	if (this->isConnected)
	{
		Aseba::UserMessage::DataVector data(values.size());
		QVectorIterator<int> it(values);
		unsigned i = 0;
		while (it.hasNext())
			data[i++] = it.next();
		Aseba::UserMessage(id, data).serialize(stream);
		stream->flush();
	}
}

// Dashel connection was closed
void DashelInterface::connectionClosed(Dashel::Stream* stream, bool abnormal)
{
		Q_UNUSED(stream);
		Q_UNUSED(abnormal);
		emit dashelDisconnection();
		this->stream = 0;
}

// internals
void DashelInterface::run()
{
	while (1)
	{
		try
		{
			stream = Dashel::Hub::connect(dashelParams.toStdString());

			emit dashelConnection();
			qDebug() << "Connected to target: " << dashelParams;
			isConnected = true;
			break;
		}
		catch (Dashel::DashelException e)
		{
			qDebug() << "Cannot connect to target: " << dashelParams;
			sleep(1000000L);	// 1s
		}

	}

	while (isRunning)
		Dashel::Hub::run();
}

