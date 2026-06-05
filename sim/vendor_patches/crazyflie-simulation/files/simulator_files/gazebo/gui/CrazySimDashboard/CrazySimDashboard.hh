#ifndef CRAZYSIM_DASHBOARD_HH_
#define CRAZYSIM_DASHBOARD_HH_

#include <mutex>
#include <string>

#include <gz/gui/Plugin.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>

#include <QString>

namespace crazyflie_interface
{
  class CrazySimDashboard : public gz::gui::Plugin
  {
    Q_OBJECT

    Q_PROPERTY(QString text READ Text NOTIFY TextChanged)
    Q_PROPERTY(QString topic READ Topic NOTIFY TopicChanged)

    public: CrazySimDashboard();
    public: ~CrazySimDashboard() override = default;

    public: QString Text() const;
    public: QString Topic() const;

    protected: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    private slots: void SetText(const QString &_text);

    signals: void TextChanged();
    signals: void TopicChanged();

    private: void OnDashboardMessage(const gz::msgs::StringMsg &_msg);

    private: mutable std::mutex mutex;
    private: QString text;
    private: QString topic;
    private: gz::transport::Node node;
  };
}

#endif
