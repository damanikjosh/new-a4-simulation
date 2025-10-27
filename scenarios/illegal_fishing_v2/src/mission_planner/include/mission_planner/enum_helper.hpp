#ifndef MISSION_PLANNER__ENUM_HELPER_HPP__
#define MISSION_PLANNER__ENUM_HELPER_HPP__

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

const char * connection_result_str(mavsdk::ConnectionResult r) {
    switch (r) {
    case mavsdk::ConnectionResult::Success: return "Success";
    case mavsdk::ConnectionResult::Timeout: return "Timeout";
    case mavsdk::ConnectionResult::SocketError: return "SocketError";
    case mavsdk::ConnectionResult::BindError: return "BindError";
    case mavsdk::ConnectionResult::SocketConnectionError: return "SocketConnectionError";
    case mavsdk::ConnectionResult::ConnectionError: return "ConnectionError";
    case mavsdk::ConnectionResult::NotImplemented: return "NotImplemented";
    case mavsdk::ConnectionResult::SystemNotConnected: return "SystemNotConnected";
    case mavsdk::ConnectionResult::SystemBusy: return "SystemBusy";
    case mavsdk::ConnectionResult::CommandDenied: return "CommandDenied";
    case mavsdk::ConnectionResult::DestinationIpUnknown: return "DestinationIpUnknown";
    case mavsdk::ConnectionResult::ConnectionsExhausted: return "ConnectionsExhausted";
    case mavsdk::ConnectionResult::ConnectionUrlInvalid: return "ConnectionUrlInvalid";
    case mavsdk::ConnectionResult::BaudrateUnknown: return "BaudrateUnknown";
    default: return "UnknownConnectionResult";
    }
}

const char * vehicle_type_str(int vehicle_type) {
    switch (vehicle_type) {
    case 0: return "UAV";
    case 1: return "USV";
    case 2: return "ENEMY";
    default: return "UNKNOWN";
    }
}

const char * flight_mode_str(mavsdk::Telemetry::FlightMode flight_mode) {
    switch (flight_mode) {
    case mavsdk::Telemetry::FlightMode::Unknown: return "Unknown";
    case mavsdk::Telemetry::FlightMode::Ready: return "Ready";
    case mavsdk::Telemetry::FlightMode::Takeoff: return "Takeoff";
    case mavsdk::Telemetry::FlightMode::Hold: return "Hold";
    case mavsdk::Telemetry::FlightMode::Mission: return "Mission";
    case mavsdk::Telemetry::FlightMode::ReturnToLaunch: return "ReturnToLaunch";
    case mavsdk::Telemetry::FlightMode::Land: return "Land";
    case mavsdk::Telemetry::FlightMode::Offboard: return "Offboard";
    case mavsdk::Telemetry::FlightMode::FollowMe: return "FollowMe";
    case mavsdk::Telemetry::FlightMode::Manual: return "Manual";
    case mavsdk::Telemetry::FlightMode::Altctl: return "Altctl";
    case mavsdk::Telemetry::FlightMode::Posctl: return "Posctl";
    case mavsdk::Telemetry::FlightMode::Acro: return "Acro";
    case mavsdk::Telemetry::FlightMode::Stabilized: return "Stabilized";
    case mavsdk::Telemetry::FlightMode::Rattitude: return "Rattitude";
    default: return "UnknownFlightMode";
    }
}

// enum class Result {
//     Unknown, /**< @brief Unknown result. */
//     Success, /**< @brief Request was successful. */
//     NoSystem, /**< @brief No system is connected. */
//     ConnectionError, /**< @brief Connection error. */
//     Busy, /**< @brief Vehicle is busy. */
//     CommandDenied, /**< @brief Command refused by vehicle. */
//     CommandDeniedLandedStateUnknown, /**< @brief Command refused because landed state is
//                                         unknown. */
//     CommandDeniedNotLanded, /**< @brief Command refused because vehicle not landed. */
//     Timeout, /**< @brief Request timed out. */
//     VtolTransitionSupportUnknown, /**< @brief Hybrid/VTOL transition support is unknown. */
//     NoVtolTransitionSupport, /**< @brief Vehicle does not support hybrid/VTOL transitions. */
//     ParameterError, /**< @brief Error getting or setting parameter. */
//     Unsupported, /**< @brief Action not supported. */
//     Failed, /**< @brief Action failed. */
//     InvalidArgument, /**< @brief Invalid argument. */
// };
const char * action_result_str(mavsdk::Action::Result result) {
    switch (result) {
    case mavsdk::Action::Result::Unknown: return "Unknown";
    case mavsdk::Action::Result::Success: return "Success";
    case mavsdk::Action::Result::NoSystem: return "NoSystem";
    case mavsdk::Action::Result::ConnectionError: return "ConnectionError";
    case mavsdk::Action::Result::Busy: return "Busy";
    case mavsdk::Action::Result::CommandDenied: return "CommandDenied";
    case mavsdk::Action::Result::CommandDeniedLandedStateUnknown: return "CommandDeniedLandedStateUnknown";
    case mavsdk::Action::Result::CommandDeniedNotLanded: return "CommandDeniedNotLanded";
    case mavsdk::Action::Result::Timeout: return "Timeout";
    case mavsdk::Action::Result::VtolTransitionSupportUnknown: return "VtolTransitionSupportUnknown";
    case mavsdk::Action::Result::NoVtolTransitionSupport: return "NoVtolTransitionSupport";
    case mavsdk::Action::Result::ParameterError: return "ParameterError";
    case mavsdk::Action::Result::Unsupported: return "Unsupported";
    case mavsdk::Action::Result::Failed: return "Failed";
    case mavsdk::Action::Result::InvalidArgument: return "InvalidArgument";
    default: return "UnknownActionResult";
    }
}

// Unknown	Unknown result.
// Success	Request succeeded.
// Error	Error.
// TooManyMissionItems	Too many mission items in the mission.
// Busy	Vehicle is busy.
// Timeout	Request timed out.
// InvalidArgument	Invalid argument.
// Unsupported	Mission downloaded from the system is not supported.
// NoMissionAvailable	No mission available on the system.
// UnsupportedMissionCmd	Unsupported mission command.
// TransferCancelled	Mission transfer (upload or download) has been cancelled.
// NoSystem	No system connected.
// Next	Intermediate message showing progress.
// Denied	Request denied.
// ProtocolError	There was a protocol error.
// IntMessagesNotSupported	The system does not support the MISSION_INT protocol.

const char * mission_result_str(mavsdk::Mission::Result result) {
    switch (result) {
    case mavsdk::Mission::Result::Unknown: return "Unknown";
    case mavsdk::Mission::Result::Success: return "Success";
    case mavsdk::Mission::Result::Error: return "Error";
    case mavsdk::Mission::Result::TooManyMissionItems: return "TooManyMissionItems";
    case mavsdk::Mission::Result::Busy: return "Busy";
    case mavsdk::Mission::Result::Timeout: return "Timeout";
    case mavsdk::Mission::Result::InvalidArgument: return "InvalidArgument";
    case mavsdk::Mission::Result::Unsupported: return "Unsupported";
    case mavsdk::Mission::Result::NoMissionAvailable: return "NoMissionAvailable";
    case mavsdk::Mission::Result::UnsupportedMissionCmd: return "UnsupportedMissionCmd";
    case mavsdk::Mission::Result::TransferCancelled: return "TransferCancelled";
    case mavsdk::Mission::Result::NoSystem: return "NoSystem";
    case mavsdk::Mission::Result::Next: return "Next";
    case mavsdk::Mission::Result::Denied: return "Denied";
    case mavsdk::Mission::Result::ProtocolError: return "ProtocolError";
    case mavsdk::Mission::Result::IntMessagesNotSupported: return "IntMessagesNotSupported";
    default: return "UnknownMissionResult";
    }
}


#endif  // MISSION_PLANNER__ENUM_HELPER_HPP__