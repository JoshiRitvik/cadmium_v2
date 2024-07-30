#ifndef CADMIUM_MODELING_DEVS_FMI_HPP_
#define CADMIUM_MODELING_DEVS_FMI_HPP_

#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <cassert>
#include "component.hpp"
#include "port.hpp"
#include "atomic.hpp"
#include "../../exception.hpp"

#include "../../../FMI_v2.0/fmi2Functions.h"
#include "../../../FMI_v2.0/fmi2FunctionTypes.h"
#include "../../../FMI_v2.0/fmi2TypesPlatform.h"

//review
#include <dlfcn.h>
#define OPEN_LIB(name) dlopen(name,RTLD_LAZY)
#define GET_FUNC(hndl,name) dlsym(hndl,name)
#define CLOSE_LIB(hndl) dlclose(hndl)

typedef enum{
    continuous_time_mode,
    event_mode,
}eFMI_mode;

namespace cadmium {

template <typename S> class FMI:
    public Atomic<S>
    {
        public:

        FMI(const std::string& id, 
            S initialState,
            const char* modelname,
			const char* guid,
			const char* resource_path,
			// int num_state_variables,		
			// int num_event_indicators,	
			const char* so_file_name,
			// const double tolerance = 1E-06, //1E-8	
			// int num_extra_event_indicators = 0,		
			double start_time = 0.0
            );

        virtual void internalTransition(S& s) const = 0; //

        virtual void externalTransition(S& s, double e) const = 0;

        virtual void output(const S& s) const = 0;

        virtual double timeAdvance(const S& s) const = 0;

        // virtual void confluentTransition(S& s, double e) const = 0;	

        virtual void initialize(S& s);

		//virtual void update_state_variables(S& s) const; //S& s

        //virtual void iterate_event(S& s) const;
        // Get the current time
		//double get_time() const { return current_time; }
		// Get the value of a real variable
		double get_real(int k) const;
		// Set the value of a real variable
		void set_real(int k, double val) const;
		// Get the value of an integer variable
		int get_int(int k) const;
		// Set the value of an integer variable
		void set_int(int k, int val) const;
		// Get the value of a boolean variable
		bool get_bool(int k) const;
		// Set the value of a boolean variable
		void set_bool(int k, bool val) const;
		// advancing the simulation time to next communication step 
		void do_step(double CurrentTime, double h) const;

        static void FMI_logging(
			fmi2ComponentEnvironment componentEnvironment,
			fmi2String instanceName,
			fmi2Status status,
			fmi2String category,
			fmi2String message,...)
		{
			if (message != NULL){			
		
				fprintf(stderr, message,"\n");
			}
		}

		fmi2CallbackFunctions* callbackFunction;
		
        private:
		//eFMI_mode FMI_mode;
        double current_time;
        //double next_event_time;
        double *x;
        double no_of_states;
        void* so_hndl;
        //review
		// Reference to the FMI
		fmi2Component m;
		// Pointer to the FMI interface
		fmi2Component (*_fmi2Instantiate)(fmi2String, fmi2Type,
				fmi2String, fmi2String, const fmi2CallbackFunctions*,
				fmi2Boolean, fmi2Boolean);
		void (*_fmi2FreeInstance)(fmi2Component);
		fmi2Status (*_fmi2SetupExperiment)(fmi2Component, fmi2Boolean,
				fmi2Real, fmi2Real, fmi2Boolean, fmi2Real);
		fmi2Status (*_fmi2EnterInitializationMode)(fmi2Component);
		fmi2Status (*_fmi2ExitInitializationMode)(fmi2Component);
		fmi2Status (*_fmi2GetReal)(fmi2Component, const fmi2ValueReference*, size_t, fmi2Real*);
		fmi2Status (*_fmi2GetInteger)(fmi2Component, const fmi2ValueReference*, size_t, fmi2Integer*);
		fmi2Status (*_fmi2GetBoolean)(fmi2Component, const fmi2ValueReference*, size_t, fmi2Boolean*);
		fmi2Status (*_fmi2GetString)(fmi2Component, const fmi2ValueReference*, size_t, fmi2String*);
		fmi2Status (*_fmi2SetReal)(fmi2Component, const fmi2ValueReference*, size_t, const fmi2Real*);
		fmi2Status (*_fmi2SetInteger)(fmi2Component, const fmi2ValueReference*, size_t, const fmi2Integer*);
		fmi2Status (*_fmi2SetBoolean)(fmi2Component, const fmi2ValueReference*, size_t, const fmi2Boolean*);
		fmi2Status (*_fmi2SetString)(fmi2Component, const fmi2ValueReference*, size_t, const fmi2String*);

		/*
		fmi2Status (*_fmi2EnterEventMode)(fmi2Component);
		fmi2Status (*_fmi2NewDiscreteStates)(fmi2Component,fmi2EventInfo*);
		fmi2Status (*_fmi2EnterContinuousTimeMode)(fmi2Component);
		fmi2Status (*_fmi2CompletedIntegratorStep)(fmi2Component, fmi2Boolean, fmi2Boolean*, fmi2Boolean*);
		fmi2Status (*_fmi2SetTime)(fmi2Component, fmi2Real);
		fmi2Status (*_fmi2SetContinuousStates)(fmi2Component, const fmi2Real*, size_t);
		fmi2Status (*_fmi2GetDerivatives)(fmi2Component, fmi2Real*, size_t);
		fmi2Status (*_fmi2GetEventIndicators)(fmi2Component, fmi2Real*, size_t);
		fmi2Status (*_fmi2GetContinuousStates)(fmi2Component, fmi2Real*, size_t);
		*/

		fmi2Status (*_fmi2DoStep)(fmi2Component,fmi2Real,fmi2Real,fmi2Boolean);

    };

template <typename S>
FMI<S>::FMI(const std::string& id, 
            S initialState,
            const char* modelname,
			const char* guid,
			const char* resource_path,
			// int num_state_variables,		
			// int num_event_indicators,	
			const char* so_file_name,
			// const double tolerance,		
			// int num_extra_event_indicators, 
			 double start_time):
            Atomic<S>(id,initialState),
            current_time(start_time),
			so_hndl (NULL)
            // no_of_states(num_state_variables)	
			//next_event_time(std::numeric_limits<double>::infinity())
{
	fmi2CallbackFunctions callbackvar = {cadmium::FMI<S>::FMI_logging,calloc,free,NULL,NULL};
	callbackFunction = new fmi2CallbackFunctions(callbackvar);
	so_hndl = OPEN_LIB(so_file_name);
	if (so_hndl == NULL)
	{
		throw::cadmium::CadmiumModelException("Could not load so file");
    }
	// This only works with a POSIX compliant compiler/system
	_fmi2Instantiate = (fmi2Component (*)(fmi2String, fmi2Type,
		fmi2String, fmi2String, const fmi2CallbackFunctions*,
		fmi2Boolean, fmi2Boolean))GET_FUNC(so_hndl,"fmi2Instantiate");
	assert(_fmi2Instantiate != NULL);
	_fmi2FreeInstance = (void (*)(fmi2Component))GET_FUNC(so_hndl,"fmi2FreeInstance");
	assert(_fmi2FreeInstance != NULL);
	_fmi2SetupExperiment = (fmi2Status (*)(fmi2Component, fmi2Boolean,
		fmi2Real, fmi2Real, fmi2Boolean, fmi2Real))GET_FUNC(so_hndl,"fmi2SetupExperiment");
	assert(_fmi2SetupExperiment != NULL);
	_fmi2EnterInitializationMode = (fmi2Status (*)(fmi2Component))GET_FUNC(so_hndl,"fmi2EnterInitializationMode");
	assert(_fmi2EnterInitializationMode != NULL);
	_fmi2ExitInitializationMode = (fmi2Status (*)(fmi2Component))GET_FUNC(so_hndl,"fmi2ExitInitializationMode");
	assert(_fmi2ExitInitializationMode != NULL);
	_fmi2GetReal = (fmi2Status (*)(fmi2Component, const fmi2ValueReference*, size_t, fmi2Real*))
		GET_FUNC(so_hndl,"fmi2GetReal");
	assert(_fmi2GetReal != NULL);
	_fmi2GetInteger = (fmi2Status (*)(fmi2Component, const fmi2ValueReference*, size_t, fmi2Integer*)) 
		GET_FUNC(so_hndl,"fmi2GetInteger");
	assert(_fmi2GetInteger != NULL);
	_fmi2GetBoolean = (fmi2Status (*)(fmi2Component, const fmi2ValueReference*, size_t, fmi2Boolean*))
		GET_FUNC(so_hndl,"fmi2GetBoolean");
	assert(_fmi2GetBoolean != NULL);
	_fmi2GetString = (fmi2Status (*)(fmi2Component, const fmi2ValueReference*, size_t, fmi2String*))
		GET_FUNC(so_hndl,"fmi2GetString");
	assert(_fmi2GetString != NULL);
	_fmi2SetReal = (fmi2Status (*)(fmi2Component, const fmi2ValueReference*, size_t, const fmi2Real*))
		GET_FUNC(so_hndl,"fmi2SetReal");
	assert(_fmi2SetReal != NULL);
	_fmi2SetInteger = (fmi2Status (*)(fmi2Component, const fmi2ValueReference*, size_t, const fmi2Integer*))
		GET_FUNC(so_hndl,"fmi2SetInteger");
	assert(_fmi2SetInteger != NULL);
	_fmi2SetBoolean = (fmi2Status (*)(fmi2Component, const fmi2ValueReference*, size_t, const fmi2Boolean*))
		GET_FUNC(so_hndl,"fmi2SetBoolean");
	assert(_fmi2SetBoolean != NULL);
	_fmi2SetString = (fmi2Status (*)(fmi2Component, const fmi2ValueReference*, size_t, const fmi2String*))
		GET_FUNC(so_hndl,"fmi2SetString");
	assert(_fmi2SetString != NULL);

	/*
	_fmi2EnterEventMode = (fmi2Status (*)(fmi2Component))GET_FUNC(so_hndl,"fmi2EnterEventMode");
	assert(_fmi2EnterEventMode != NULL);
	_fmi2NewDiscreteStates = (fmi2Status (*)(fmi2Component,fmi2EventInfo*))GET_FUNC(so_hndl,"fmi2NewDiscreteStates");
	assert(_fmi2NewDiscreteStates != NULL);
	_fmi2EnterContinuousTimeMode = (fmi2Status (*)(fmi2Component))GET_FUNC(so_hndl,"fmi2EnterContinuousTimeMode");
	assert(_fmi2EnterContinuousTimeMode != NULL);
	_fmi2CompletedIntegratorStep = (fmi2Status (*)(fmi2Component, fmi2Boolean, fmi2Boolean*, fmi2Boolean*))
		GET_FUNC(so_hndl,"fmi2CompletedIntegratorStep");
	assert(_fmi2CompletedIntegratorStep != NULL);
	_fmi2SetTime = (fmi2Status (*)(fmi2Component, fmi2Real))GET_FUNC(so_hndl,"fmi2SetTime");
	assert(_fmi2SetTime != NULL);
	_fmi2SetContinuousStates = (fmi2Status (*)(fmi2Component, const fmi2Real*, size_t))
		GET_FUNC(so_hndl,"fmi2SetContinuousStates");
	assert(_fmi2SetContinuousStates != NULL);
	_fmi2GetDerivatives = (fmi2Status (*)(fmi2Component, fmi2Real*, size_t))GET_FUNC(so_hndl,"fmi2GetDerivatives");
	assert(_fmi2GetDerivatives != NULL);
	_fmi2GetEventIndicators = (fmi2Status (*)(fmi2Component, fmi2Real*, size_t))GET_FUNC(so_hndl,"fmi2GetEventIndicators");
	assert(_fmi2GetEventIndicators != NULL);
	_fmi2GetContinuousStates = (fmi2Status (*)(fmi2Component, fmi2Real*, size_t))GET_FUNC(so_hndl,"fmi2GetContinuousStates");
	assert(_fmi2GetContinuousStates != NULL);
	*/
	_fmi2DoStep = (fmi2Status (*)(fmi2Component, fmi2Real, fmi2Real, fmi2Boolean))GET_FUNC(so_hndl,"fmi2DoStep");
	assert(_fmi2DoStep != NULL);
	// Create the FMI component
	//m = _fmi2Instantiate(modelname,fmi2ModelExchange,guid,"",callbackFunction,fmi2False,fmi2False); //remove comment for FMI model exchange and comment next line
	m = _fmi2Instantiate(modelname,fmi2CoSimulation,guid,resource_path,callbackFunction,fmi2False,fmi2False); 
    std::cout << m << "FMI instantiated" << std::endl;
	assert(m != NULL);
	/*
	fmi2Status fmi_status = _fmi2SetTime(m,0);
    if(fmi_status != fmi2OK)
    {
        std::cout << &_fmi2SetTime << std::endl;
        throw::cadmium::CadmiumModelException("_fmi2SetTime failed");
    }
	_fmi2SetupExperiment(m,fmi2False ,0.0,current_time,fmi2True,10);
	*/

    fmi2Status fmi_status = _fmi2SetupExperiment(m,fmi2False,0.0,-1.0,fmi2False,-1.0); //
	
}
/*
template <typename S>
void FMI<S>::iterate_event(S& s) const
{
    fmi2Status fmi_status;
    fmi2EventInfo eventInfo;
	
	do
	{
		fmi_status = _fmi2NewDiscreteStates(m,&eventInfo);
		std::cout << " waiting for state " << " " << eventInfo.newDiscreteStatesNeeded << std::endl;
		if(fmi_status != fmi2OK)
        {
            throw::cadmium::CadmiumModelException("_fmi2NewDiscreteStates failed");
        }
	}
	while (eventInfo.newDiscreteStatesNeeded == fmi2True);
	if (eventInfo.nextEventTimeDefined == fmi2True)
	{
		s.next_event_time = eventInfo.nextEventTime;
		std::cout << " timed event "<< std::endl;
	}

		std::cout << "no timed event "<< eventInfo.valuesOfContinuousStatesChanged  << " " << eventInfo.newDiscreteStatesNeeded << " " << eventInfo.nominalsOfContinuousStatesChanged  <<" " << eventInfo.nextEventTime << " " << eventInfo.nextEventTimeDefined << std::endl;
	if(fmi_status != fmi2OK)
    {
        throw::cadmium::CadmiumModelException("iterate_event failed");
    }
} 
*/
template <typename S>
void FMI<S>::initialize(S& s) 
{
    fmi2Status fmi_status;

	/*
	fmi_status = _fmi2SetTime(m,current_time);
    if(fmi_status != fmi2OK)
    {
        std::cout << &_fmi2SetTime << std::endl;
        throw::cadmium::CadmiumModelException("_fmi2SetTime failed");
    }
	*/

    fmi_status = _fmi2EnterInitializationMode(m);
    if(fmi_status != fmi2OK)
    {
        throw::cadmium::CadmiumModelException("_fmi2EnterInitializationMode failed");
    }
    fmi_status = _fmi2ExitInitializationMode(m);
    if(fmi_status != fmi2OK)
    {
        throw::cadmium::CadmiumModelException("_fmi2ExitInitializationMode failed");
    }
	
	/*
	fmi_status = _fmi2SetTime(m,0.0);
     if(fmi_status != fmi2OK)
    {
        std::cout << &_fmi2SetTime << std::endl;
        throw::cadmium::CadmiumModelException("_fmi2SetTime failed");
    }
		{
		fmi_status = _fmi2EnterEventMode(m);
        if(fmi_status != fmi2OK)
        {
            throw::cadmium::CadmiumModelException("_fmi2EnterEventMode failed");
        }
		s.FMI_mode = event_mode;
	}
    iterate_event(s);
    fmi_status = _fmi2EnterContinuousTimeMode(m);
    if(fmi_status != fmi2OK)
    {
        throw::cadmium::CadmiumModelException("_fmi2EnterContinuousTimeMode failed");
    }

    fmi_status = _fmi2GetContinuousStates(m,&s.x_value,no_of_states);
    if(fmi_status != fmi2OK)
    {
        throw::cadmium::CadmiumModelException("_fmi2GetContinuousStates failed");
    }
    //set time
	std::cout << " next_event_time "<< no_of_states << " " << s.x_value << " " <<fmi_status << std::endl;
    s.FMI_mode = continuous_time_mode;
	*/

}
/*
template <typename S>
void FMI<S>::internalTransition(S& s) const
{
    fmi2Status fmi_status;
	fmi2Boolean enterEventMode,terminateSimulation;
	const fmi2ValueReference ref = 2;
	fmi_status = _fmi2SetTime(m,s.continuous_state_time);
	if(fmi_status != fmi2OK)
    {
        throw::cadmium::CadmiumModelException("_fmi2SetTime failed");
    }
	
	fmi_status = _fmi2SetReal(m,&ref,1,&s.input_val); //set values only for test
	if(fmi_status != fmi2OK)
    {
        throw::cadmium::CadmiumModelException("_fmi2SetReal failed");
    }

	fmi_status = _fmi2SetContinuousStates(m,&s.x_value, no_of_states);
	if(fmi_status != fmi2OK)
    {
        throw::cadmium::CadmiumModelException("_fmi2SetContinuousStates failed");
    }

	//detect event

	fmi_status = _fmi2CompletedIntegratorStep(m,fmi2True,&enterEventMode,&terminateSimulation);
	if(fmi_status != fmi2OK)
    {
        throw::cadmium::CadmiumModelException("_fmi2SetContinuousStates failed");
    }

	//handle event
	//if ((s.FMI_mode == continuous_time_mode) || (enterEventMode))
	{
		fmi_status = _fmi2EnterEventMode(m);
        if(fmi_status != fmi2OK)
        {
            throw::cadmium::CadmiumModelException("_fmi2EnterEventMode failed");
        }
		s.FMI_mode = event_mode;
	}
	// Process events
	iterate_event(s);

	fmi_status = _fmi2EnterContinuousTimeMode(m);
    if(fmi_status != fmi2OK)
    {
        throw::cadmium::CadmiumModelException("_fmi2EnterContinuousTimeMode failed");
    }
	// Update the state variable array
	fmi_status = _fmi2GetContinuousStates(m,&s.x_value,no_of_states);
    if(fmi_status != fmi2OK)
    {
        throw::cadmium::CadmiumModelException("_fmi2GetContinuousStates failed");
    }

}

template <typename S>
void FMI<S>::update_state_variables(S& s) const //
{
    fmi2Status fmi_status;
	if (s.FMI_mode == continuous_time_mode)
	{
		fmi_status = _fmi2EnterEventMode(m);
        if(fmi_status != fmi2OK)
        {
            throw::cadmium::CadmiumModelException("_fmi2EnterEventMode failed");
        }
		s.FMI_mode = event_mode;
	}
	// Process events
	iterate_event(s);
	// Update the state variable array
	fmi_status = _fmi2GetContinuousStates(m,x,no_of_states);
    if(fmi_status != fmi2OK)
    {
        throw::cadmium::CadmiumModelException("_fmi2GetContinuousStates failed");
    }
}

template <typename S>
void FMI<S>::externalTransition(S& s, double e) const
{
    fmi2Status fmi_status;
	if (s.FMI_mode == continuous_time_mode)
	{
		fmi_status = _fmi2EnterEventMode(m);
        if(fmi_status != fmi2OK)
        {
            throw::cadmium::CadmiumModelException("_fmi2EnterEventMode failed");
        }
		s.FMI_mode = event_mode;
	}
	// Process events
	iterate_event();
	// Update the state variable array
	fmi_status = _fmi2GetContinuousStates(m,x,no_of_states);
    if(fmi_status != fmi2OK)
    {
        throw::cadmium::CadmiumModelException("_fmi2GetContinuousStates failed");
    }
}
*/

/*
void output(const S& s)
{}

double timeAdvance(const S& s) 
{}

void confluentTransition(S& s, double e)
{}
*/
template <typename S>
double FMI<S>::get_real(int k) const
{
	const fmi2ValueReference ref = k;
	fmi2Real val;
	fmi2Status fmi_status = _fmi2GetReal(m,&ref,1,&val);
	if(fmi_status != fmi2OK)
    {
        throw::cadmium::CadmiumModelException("_fmi2GetReal failed");
    }
	return val;
}

template <typename S>
void FMI<S>::set_real(int k, double val) const
{
	const fmi2ValueReference ref = k;
	fmi2Real fmi_val = val;
	fmi2Status fmi_status = _fmi2SetReal(m,&ref,1,&fmi_val);
	if(fmi_status != fmi2OK)
    {
        throw::cadmium::CadmiumModelException("_fmi2SetReal failed");
    }
}

template <typename S>
int FMI<S>::get_int(int k) const
{
	const fmi2ValueReference ref = k;
	fmi2Integer val;
	fmi2Status fmi_status = _fmi2GetInteger(m,&ref,1,&val);
	if(fmi_status != fmi2OK)
    {
        throw::cadmium::CadmiumModelException("_fmi2GetInteger failed");
    }
	return val;
}

template <typename S>
void FMI<S>::set_int(int k, int val) const
{
	const fmi2ValueReference ref = k;
	fmi2Integer fmi_val = val;
	fmi2Status fmi_status = _fmi2SetInteger(m,&ref,1,&fmi_val);
	if(fmi_status != fmi2OK)
    {
        throw::cadmium::CadmiumModelException("_fmi2SetInteger failed");
    }
}

template <typename S>
bool FMI<S>::get_bool(int k) const
{
	const fmi2ValueReference ref = k;
	fmi2Boolean val;
	fmi2Status fmi_status = _fmi2GetBoolean(m,&ref,1,&val);
	if(fmi_status != fmi2OK)
    {
        throw::cadmium::CadmiumModelException("_fmi2GetBoolean failed");
    }
	return (val == fmi2True);
}

template <typename S>
void FMI<S>::set_bool(int k, bool val) const
{
	const fmi2ValueReference ref = k;
	fmi2Boolean fmi_val = fmi2False;
	if (val) fmi_val = fmi2True;
	fmi2Status fmi_status = _fmi2SetBoolean(m,&ref,1,&fmi_val);
	if(fmi_status != fmi2OK)
    {
        throw::cadmium::CadmiumModelException("_fmi2SetBoolean failed");
    }
}

template <typename S>
void FMI<S>::do_step(double CurrentTime, double h) const
{
	fmi2Status fmi_status = _fmi2DoStep(m,CurrentTime,h,fmi2True);
	if(fmi_status != fmi2OK)
    {
        throw::cadmium::CadmiumModelException("_fmi2DoStep failed");
    }
}
}
#endif //CADMIUM_MODELING_DEVS_FMI_HPP_