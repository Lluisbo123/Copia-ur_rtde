#ifndef robotiq_gripperH
#define robotiq_gripperH
//============================================================================
/// \file   robotiq_gripper.h
/// \author Uwe Kindler
/// \date   31.10.2020
/// \brief  Declaration of RobotiqGripper
//============================================================================

//============================================================================
//                                   INCLUDES
//============================================================================
#include <string>
#include <vector>
#include <memory>
#include <tuple>
#include <boost/asio/connect.hpp>
#include <boost/asio/socket_base.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/io_service.hpp>

namespace ur_rtde
{

/**
 * C++ driver for Robot IQ grippers
 * Communicates with the gripper directly, via socket with string commands,
 * leveraging string names for variables.
 * WRITE VARIABLES (CAN ALSO READ)
 * ACT = 'ACT'  # act : activate (1 while activated, can be reset to clear fault status)
 * GTO = 'GTO'  # gto : go to (will perform go to with the actions set in pos, for, spe)
 * ATR = 'ATR'  # atr : auto-release (emergency slow move)
 * ADR = 'ADR'  # adr : auto-release direction (open(1) or close(0) during auto-release)
 * FOR = 'FOR'  # for : force (0-255)
 * SPE = 'SPE'  # spe : speed (0-255)
 * POS = 'POS'  # pos : position (0-255), 0 = open
	# READ VARIABLES
 * STA = 'STA'  # status (0 = is reset, 1 = activating, 3 = active)
 * PRE = 'PRE'  # position request (echo of last commanded position)
 * OBJ = 'OBJ'  # object detection (0 = moving, 1 = outer grip, 2 = inner grip, 3 = no object at rest)
 * FLT = 'FLT'  # fault (0=ok, see manual for errors if not zero)
 */
class RobotiqGripper
{
public:
	using VariableDict = std::vector<std::pair<std::string, int>>;
	/**
	 * Gripper status reported by the gripper.
	 * The integer values have to match what the gripper sends.
	 */
    enum eStatus
    {
		RESET = 0,     //!< RESET
		ACTIVATING = 1,//!< ACTIVATING
		// UNUSED = 2  # This value is currently not used by the gripper firmware
		ACTIVE = 3,    //!< ACTIVE
    };

    /**
     *  Object status reported by the gripper.
     *  The integer values have to match what the gripper sends.
     */
    enum eObjectStatus
    {
        MOVING = 0,
        STOPPED_OUTER_OBJECT = 1,
        STOPPED_INNER_OBJECT = 2,
        AT_DEST = 3,
    };

    /**
     * Connection status
     */
	enum class ConnectionState : std::uint8_t
	{
		DISCONNECTED = 0,
		CONNECTED = 1,
	};

	enum eMoveMode
	{
		START_MOVE,
		WAIT_FINISHED
	};

	/**
	 * Constructor - creates a RobotiqGripper object with the given Hostname/IP
	 * and Port.
	 * @param Hostname The hostname or ip address to use for connection
	 * @param Port The port to use for connection
	 * @param verbose Prints additional debug information if true
	 */
	RobotiqGripper(const std::string& Hostname, int Port = 63352, bool verbose = false);

	/**
	 * Connects to the gripper server
	 */
	void connect();

	/**
	 * Disconnects from the gripper server
	 */
	void disconnect();

	/**
	 * Returns true if connected
	 */
	bool isConnected() const;

	/**
	 * Resets the activation flag in the gripper, and sets it back to one,
	 * clearing previous fault flags.
	 * This is required after an emergency stop.
	 * \param auto_calibrate: Whether to calibrate the minimum and maximum
	 * positions based on actual motion.
	 */
	void activate(bool auto_calibrate = false);

	/**
	 * Attempts to calibrate the open and closed positions, by slowly closing
	 * and opening the gripper.
	 */
	void autoCalibrate();

	/**
	 * Returns whether the gripper is active
	 */
	bool isActive();

	/**
	 * Returns the minimum position the gripper can reach (open position)
	 * in the range from 0 - 255
	 */
    int getMinPosition() const;

    /**
     * Returns the maximum position the gripper can reach (closed position)
     * in the range from 0 - 255
     */
    int getMaxPosition() const;

    /**
     * Returns the open position
     */
    int getOpenPosition() const;

    /**
     * Returns what is considered the closed position for gripper (maximum position value).
     */
    int getClosedPosition() const;

    /**
     * Returns the current position as returned by the physical hardware.
     */
    int getCurrentPosition();

    /**
     * Returns whether the current position is considered as being fully open.
     */
    bool isOpen();

    /**
     * Returns whether the current position is considered as being fully closed.
     */
    bool isClosed();

    /**
     * \brief Sends commands to start moving towards the given position,
     * with the specified speed and force.
     * \param Position: Position to move to [min_position, max_position]
     * \param Speed: Speed to move at [min_speed, max_speed]
     * \param Force: Force to use [min_force, max_force]
     * \return: An integer with the actual position that was
     *  requested, after being adjusted to the min/max calibrated range.
     */
	int move(int Position, int Speed, int Force = 0, eMoveMode MoveMode = START_MOVE);

	int moveNorm(float Position, float NormSpeed, float NormForce = 0.0, eMoveMode MoveMode = START_MOVE);

	int open(float NormSpeed, float NormForce = 0.0, eMoveMode MoveMode = START_MOVE);

	int close(float NormSpeed, float NormForce = 0.0, eMoveMode MoveMode = START_MOVE);

	/**
	 * Returns the current move status if a move is active.
	 * Use this function for polling the state.
	 */
	eObjectStatus objectDetectionStatus();

	/**
	 * Call this function after a move command to wait for completion of the
	 * commanded move.
	 */
	eObjectStatus waitForMotionComplete();

private:
	/**
	 * Print all variables for debugging
	 */
	void dumpVars();

	/**
	 * Receive function to receive data from the gripper server
	 */
	std::string receive();

	/**
	 * Send function to send data to the gripper
	 */
	void send(const std::string &str);

	/**
	 * Sends the appropriate command via socket to set the value of n variables,
	 * and waits for its 'ack' response.
	 * \param Vars Dictionary of variables to set (variable_name, value).
     * \return: True on successful reception of ack, false if no ack was
     * 		received, indicating the set may not have been effective.
	 */
	bool setVars(const std::vector<std::pair<std::string, int>> Vars);

	/**
	 * Sends the appropriate command via socket to set the value of a variable,
	 * and waits for its 'ack' response.
	 * \param Var: Variable to set.
     * \param Value: Value to set for the variable.
     * \return: True on successful reception of ack, false if no ack was received,
     * indicating the set may not have been effective.
	 */
	bool setVar(const std::string& Var, int Value);

	/**
	 * Sends the appropriate command to retrieve the value of a variable from
	 * the gripper, blocking until the response is received or the socket times out.
	 * \param var: Name of the variable to retrieve.
     * \return: Value of the variable as integer.
	 */
	int getVar(const std::string& var);

	/**
	 * Resets the gripper.
	 */
	void reset();

	std::string hostname_;
	int port_;
	bool verbose_;
	ConnectionState conn_state_;
	std::shared_ptr<boost::asio::io_service> io_service_;
	std::shared_ptr<boost::asio::ip::tcp::socket> socket_;
	std::shared_ptr<boost::asio::ip::tcp::resolver> resolver_;
    int min_position_ = 0;
    int max_position_ = 255;
    int min_speed_ = 0;
    int max_speed_ = 255;
    int min_force_ = 0;
    int max_force_ = 255;
};
} // namespace ur_rtde

//---------------------------------------------------------------------------
#endif // robotiq_gripperH
