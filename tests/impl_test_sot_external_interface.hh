#ifndef _DGB_IMPL_TEST_SOT_EXTERNAL_INTEFACE_HH_
#define _DGB_IMPL_TEST_SOT_EXTERNAL_INTEFACE_HH_

#include <iostream>
#include <sot/core/abstract-sot-external-interface.hh>


class ImplTestSotExternalInterface : public
dynamicgraph::sot::AbstractSotExternalInterface {
public:
  ImplTestSotExternalInterface();
  virtual ~ImplTestSotExternalInterface();

  virtual void setupSetSensors(std::map<std::string,
                               dynamicgraph::sot::SensorValues> &) final;

  virtual void nominalSetSensors(std::map<std::string,
                                 dynamicgraph::sot::SensorValues> &) final;

  virtual void cleanupSetSensors(std::map<std::string,
                                 dynamicgraph::sot::SensorValues> &) final;
  virtual void getControl(std::map<std::string,
                          dynamicgraph::sot::ControlValues> &) final;
  
  virtual void setSecondOrderIntegration(void);
  
  virtual void setNoIntegration(void);

protected:
  // Named ctrl vector
  dynamicgraph::sot::ControlValues named_ctrl_vec_;

  // Named base free flyer vector
  dynamicgraph::sot::ControlValues named_base_eff_vec_;

};


#endif
