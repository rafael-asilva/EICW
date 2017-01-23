/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Rafael Araujo da Silva
 *
 * The goal of this experiment is investigate the proposed enlargement of the
 * contention window through the shortening of slot time in order to reduce the
 * probability of collisions in the backoff process.
 *
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/stats-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("CI927RAS");

//------------------------------------------------------------------------
// Collision Detection WiFi PHY Class
//------------------------------------------------------------------------
class CollissionDetectorWifiPhy: public YansWifiPhy
{
public:
  static TypeId GetTypeId (void);
  void StartReceivePreambleAndHeader (Ptr<Packet> packet, double rxPowerDbm, WifiTxVector txVector, WifiPreamble preamble, struct mpduInfo aMpdu, Time rxDuration);

private:
  Ptr<Packet> lastPacket;
  TracedCallback<Ptr<const Packet> > m_phyNormalCollisionTrace;
  TracedCallback<Ptr<const Packet> > m_phyLateCollisionTrace;
};

TypeId
CollissionDetectorWifiPhy::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::CollissionDetectorWifiPhy")
    .SetParent<YansWifiPhy> ()
    .SetGroupName ("Wifi")
    .AddConstructor<CollissionDetectorWifiPhy> ()
    .AddTraceSource ("PhyNormalCollision", "Trace source indicating a packet "
		     "has received in RX state at beginning of RX",
		     MakeTraceSourceAccessor (&CollissionDetectorWifiPhy::m_phyNormalCollisionTrace),
		     "ns3::Packet::TracedCallback")
   .AddTraceSource ("PhyLateCollision", "Trace source indicating a packet "
		    "has received in RX state in the middle of RX",
		    MakeTraceSourceAccessor (&CollissionDetectorWifiPhy::m_phyLateCollisionTrace),
		    "ns3::Packet::TracedCallback")
  ;
  return tid;
}

/*
 * This will work if you change YansWifiPhy::StartReceivePreambleAndHeader to virtual in
 * yans-wifi-phy.h
 */
void
CollissionDetectorWifiPhy::StartReceivePreambleAndHeader (Ptr<Packet> packet, double rxPowerDbm, WifiTxVector txVector, WifiPreamble preamble, struct mpduInfo aMpdu, Time rxDuration)
{
  if(this->GetEdThreshold() < rxPowerDbm + this->GetTxGain()) // RSS is strong enough to be decoded
    {
      PointerValue ptr ;
      this->GetAttribute("State", ptr);
      Ptr<WifiPhyStateHelper> wpsh = ptr.Get<WifiPhyStateHelper>();

      if(wpsh->GetState() == YansWifiPhy::IDLE)
	{
	  lastPacket = packet->Copy();	// It'll be used to address who needs to be notified about hidden nodes
	}
      else if(wpsh->GetState() == YansWifiPhy::RX)
	{
	  // A collision occurs if another packet arrives during reception
	  // (or TX but in TX we cannot identify another TX)

	  // Get Slot Time - Collisions in the same slot time are normal
	  Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (this->GetDevice());
	  Ptr<WifiMac> mac = dev->GetMac();
	  Time timeout = mac->GetSlot();

	  Time delay = Simulator::Now() - wpsh->GetLastRxStartTime();
	  if(delay.GetMicroSeconds()>timeout.GetMicroSeconds())
	    {
	      m_phyLateCollisionTrace(packet);   // A Collision after the backoff period
	    }
	  else
	    {
	      m_phyNormalCollisionTrace(packet); // A Collision into the expected backoff period
	    }
	}
    }
  YansWifiPhy::StartReceivePreambleAndHeader(packet, rxPowerDbm, txVector, preamble, aMpdu, rxDuration);
}
//------------------------------------------------------------------------
// End Collision Detection WiFi PHY Class
//------------------------------------------------------------------------

//------------------------------------------------------------------------
// NodeIDTag class to mark packets with an ID
//------------------------------------------------------------------------

// define this class in a public header
class NodeIDTag : public Tag
{
public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);
  virtual void Print (std::ostream &os) const;
  void SetSimpleValue (uint32_t value);
  uint32_t GetSimpleValue (void) const;
private:
  uint32_t m_simpleValue;
};

TypeId
NodeIDTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NodeIDTag")
    .SetParent<Tag> ()
    .AddConstructor<NodeIDTag> ()
    .AddAttribute ("NodeID",
                   "A node ID of packet sender",
                   EmptyAttributeValue (),
                   MakeUintegerAccessor (&NodeIDTag::GetSimpleValue),
                   MakeUintegerChecker<uint32_t> ())
  ;
  return tid;
}

TypeId
NodeIDTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
NodeIDTag::GetSerializedSize (void) const
{
  return 1;
}

void
NodeIDTag::Serialize (TagBuffer i) const
{
  i.WriteU8 (m_simpleValue);
}

void
NodeIDTag::Deserialize (TagBuffer i)
{
  m_simpleValue = i.ReadU32 ();
}

void
NodeIDTag::Print (std::ostream &os) const
{
  os << "v=" << (uint32_t)m_simpleValue;
}

void
NodeIDTag::SetSimpleValue (uint32_t value)
{
  m_simpleValue = value;
}

uint32_t
NodeIDTag::GetSimpleValue (void) const
{
  return m_simpleValue;
}

//------------------------------------------------------------------------
// End class NodeIDTag
//------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Node statistics class to collect data and generate statistics
//------------------------------------------------------------------------------

// Counters
const uint32_t COUNTERS_NUMBER = 13;

enum counter
{
  app_bytesTx,		// Generated Traffic at Application Layer (bytes)
  app_bytesRx, 		// Received Traffic at Application Layer (bytes)
  app_packetsTx, 	// Generated Traffic at Application Layer (packets)
  app_packetsRx, 	// Received Traffic at Application Layer (bytes)
  phy_nCollision,	// Normal Collisions
  phy_lCollision, 	// Late Collisions
  phy_RxErr, 		// Packets received with error (SNR<threshold) at PHY layer
  phy_RxOk, 		// Packets received ok at PHY layer
  mac_TxOk, 		// Tx Packets with received ACK
  mac_TxFail, 		// Tx Packets with missed ACK
  mac_TxErr, 		// Packets dropped after retry limits
  mac_FullQueueDrop, 	// Packets dropped because Queue is full
  mac_PacketTimeout	// Packets dropped with Timeout at Queue
};

const std::string counterName[COUNTERS_NUMBER] = { "appTxBytes", "appRxBytes", "appTxPackets", "appRxPackets",
    "phyNormalColl", "phyLateColl", "phyRxErr", "phyRxOk",
    "macTxOk", "macTxFail", "macTxErr", "macQueueFullDrop", "macPktTimeout"};

const std::string counterLegend[COUNTERS_NUMBER] = { "Generated Traffic", "Received Traffic", "Generated Traffic", "Received Traffic",
    "Normal Collisions", "Late Collision", "PHY Rx Errors", "PHY Rx Ok", "Received ACK", "Missed ACK", "Max Tx Retries reached",
    "MAC Queue Full Drop", "MAC Packet Timeout Drop"};

const std::string counterUnits[COUNTERS_NUMBER] = { "Bytes", "Bytes", "Number of packets", "Number of packets",
    "Collisions", "Collisions", "Dropped Packets", "Packets received", "ACK packets", "Missed ACK packets", "Dropped packets",
    "Dropped packets", "Dropped packets"};

// Ratios
const uint32_t RATIOS_NUMBER = 11;

enum ratio
{
  collision,		// Total Collisions / Total Packets Sent
  phyRxErr, 		// Packets Received with Error / Packets Received
  macTxFail, 		// Tx Packets without ACK / Tx Packets
  genPktLoss, 		// Packets not received / Total Generated Packets
  macQueueFullDrop, 	// Packets drop before enqueue / Total Generated Packets
  macPktTimeout, 	// Packets drop with timeout / Total Enqueued Packets
  macPktLoss,		// Packets drop after enqueue / Total Enqueued Packets
  txPktLoss, 		// Packets drop after retries limit / Total Tx Packets
  fairnessIdx,		// Throughput Fairness Index
  throughput,		// Received Bytes / seconds
  meanDelay		// Received Packets Mean Delay
};

const std::string ratioName[RATIOS_NUMBER] = { "collRate", "phyRxErrRate", "macTxFailRate", "appPktLoss",
    "macQueueFullRate", "macPktTimeoutRate", "macPktLoss", "txPktLoss", "fairnessIdx", "throughput", "meanDelay" };

const std::string ratioLegend[RATIOS_NUMBER] = { "Collision Rate", "Rx Error Ratio", "Tx Fail Ratio",
    "Application Packet Loss", "MAC Full Queue Drop Ratio", "Packet Timeout Ratio", "MAC PacketLoss",
    "MAC-PHY PacketLoss", "Fairness Index", "Throughput", "Mean Delay" };

const std::string ratioUnits[RATIOS_NUMBER] = { "Collisions / Total Rx Packets", "Errors / Rx Packets", "Missed ACKs / Tx Packets",
    "Dropped / Generated Packets", "Dropped / Generated Packets", "Dropped / Enqueued Packets",
    "Dropped / Enqueued Packets", "Dropped / Transmitted Packets", "Throughput Fairness Index", "Mbit/sec", "milliseconds" };

// Statistics summaries
const uint32_t SUMMARIES_NUMBER = 7;
enum summary { total, mean, stdDeviation, cov, confidenceInterval95, confidenceInterval99, median };
const std::string summary[SUMMARIES_NUMBER] = {"total", "mean", "stdDev", "C.O.V.", "C.I.95%", "C.I.99%", "median"};

class Statistics
{
public:
  Statistics ();

  // Methods to Reset Data Sample, Finish Sample, Save Sample and Import Sample
  void ResetStatistics();
  void NextSample();
  void SaveStatistics (double x, uint32_t seconds, const std::string filename);
  void ImportStatistics (double x, uint32_t seconds, std::vector<uint64_t> c[COUNTERS_NUMBER], std::vector<double> r[RATIOS_NUMBER]);

  // Application PacketSink/Rx and OnOffApp/Tx
  void AppRxCallback (std::string path, Ptr<const Packet> packet, const Address &from);
  void AppTxCallback (std::string path, Ptr<const Packet> packet);

  // YansWifiPhy - Added to YansWifiPhy::StartReceivePlcp when a packet is received during RX state
  void PhyNormalCollisionCallback(std::string context, Ptr<const Packet> packet);
  void PhyLateCollisionCallback(std::string context, Ptr<const Packet> packet);

  // Phy/State/RxError
  void PhyRxErrorCallback (std::string context, Ptr<const Packet> packet, double snr);
  void PhyRxOkCallback(std::string context, Ptr<const Packet> packet, double snr, WifiMode mode, enum WifiPreamble preamble);

  // WifiStationManager
  void MacTxOkCallback( std::string context, const WifiMacHeader &header);
  void MacTxErrCallback(std::string context, const WifiMacHeader &header);
  void MacTxDataFailedCallback(std::string context, Mac48Address address);
  void MacTxFinalDataFailedCallback(std::string context, Mac48Address address);

  // WifiMacQueue -- Added to WifiMacQueue::Enqueue when a packet is dropped 'cause queue is full
  void MacQueueFullDropCallback(std::string context, Ptr<const Packet> packet);
  void MacQueueTimeoutDropCallback(std::string context, Ptr<const Packet> packet);

  // Data to plot sample data
  Gnuplot2dDataset GetCounterDatafile(counter sample);
  Gnuplot2dDataset GetCounterDatafile(uint8_t sample);
  Gnuplot2dDataset GetRatioDatafile(ratio sample);
  Gnuplot2dDataset GetRatioDatafile(uint8_t sample);

private:

  std::vector<uint64_t> counters[COUNTERS_NUMBER];
  std::vector<double> ratios[RATIOS_NUMBER];
  double summaryC[SUMMARIES_NUMBER][COUNTERS_NUMBER];	// Summary Statistics of Counters
  double summaryR[SUMMARIES_NUMBER][RATIOS_NUMBER];	// Summary Statistics of Ratios/Proportions

  std::map<uint32_t, uint32_t>  pktCounterPerSender;	// A counter to differentiate sender and calculate Fairness
  DelayJitterEstimation delay_jitter;			// A helper to calculate the Delay of each packet
  uint64_t totalDelay;					// A counter to aid on mean delay calculation

  uint32_t size(void);				// Number of samples
  void CalcSummary(void);			// Calculate Summary after All data was collected on sample
  void CalcRatios(void);			// Calculate Ratios based on counters after each sample
  void ExportCSV(std::string filename);		// Save all data on file (to reuse)
  void PlotStatistics (double x);		// Put sample data on GNUPLOT data format (Gnuplot2dDataset)

  Gnuplot2dDataset m_output_counter[COUNTERS_NUMBER];
  Gnuplot2dDataset m_output_ratio[RATIOS_NUMBER];
};

Statistics::Statistics ()
{
  m_output_counter[app_bytesTx].SetTitle("Generated Traffic for Transmission in Bytes");
  m_output_counter[app_bytesRx].SetTitle("Received Bytes at Application Layer");
  m_output_counter[app_packetsTx].SetTitle("Generated Traffic for Transmission in Packets");
  m_output_counter[app_packetsRx].SetTitle("Received Packets at Application Layer");
  m_output_counter[phy_nCollision].SetTitle("Normal Collisions");
  m_output_counter[phy_lCollision].SetTitle("Late Collisions");
  m_output_counter[phy_RxErr].SetTitle("Rx Error at PHY Layer");
  m_output_counter[phy_RxOk].SetTitle("Rx Ok at PHY Layer");
  m_output_counter[mac_TxErr].SetTitle("Tx Error/Drop (retries>RCL)");
  m_output_counter[mac_TxOk].SetTitle("Tx Ok at MAC Layer");
  m_output_counter[mac_TxFail].SetTitle("Tx Fail (Missed Ack)");
  m_output_counter[mac_FullQueueDrop].SetTitle("Full Queue Drop");
  m_output_counter[mac_PacketTimeout].SetTitle("Packet Timeout Drop");

  m_output_ratio[collision].SetTitle("Collisions Rate");
  m_output_ratio[phyRxErr].SetTitle("PHY Rx Error Rate");
  m_output_ratio[macTxFail].SetTitle("Tx Fail/Error Rate (Missed ACK)");
  m_output_ratio[genPktLoss].SetTitle("Total Packet Loss (Not Received/Generated)");
  m_output_ratio[macQueueFullDrop].SetTitle("Full Queue Drop Ratio (Drop/Generated)");
  m_output_ratio[macPktTimeout].SetTitle("Packet Timeout Ratio (Drop/Queued)");
  m_output_ratio[macPktLoss].SetTitle("Queued Packet Loss (Not Received/Queued)");
  m_output_ratio[txPktLoss].SetTitle("Tx Packet Loss (Dropped/Transmitted)");
  m_output_ratio[fairnessIdx].SetTitle("Throughput Fairness Index");
  m_output_ratio[throughput].SetTitle("Throughput Mbits/s");
  m_output_ratio[meanDelay].SetTitle("Mean Delay (Milliseconds)");

  for(uint32_t i=0; i<COUNTERS_NUMBER; i++) m_output_counter[i].SetErrorBars(Gnuplot2dDataset::Y);
  for(uint32_t i=0; i<RATIOS_NUMBER; i++) m_output_ratio[i].SetErrorBars(Gnuplot2dDataset::Y);

  totalDelay = 0; //totalJitter = 0;
}

uint32_t
Statistics::size(void)
{
  return counters[0].size(); // Get the number of samples
}

void
Statistics::NextSample()
{
  // Compute ratios
  if(this->size() > 0) this->CalcRatios();

  // Setup for next sample to be collected
  // Add one level to each counter vector
  uint32_t size = this->size() + 1;
  for(uint32_t i=0; i<COUNTERS_NUMBER;i++) { counters[i].resize(size); }
  for(uint32_t i=0; i<RATIOS_NUMBER;i++) { ratios[i].resize(size); }

  // Reset internal counters
  totalDelay = 0;
  pktCounterPerSender.clear();
}

void
Statistics::ResetStatistics()
{
  for(uint32_t i=0; i<COUNTERS_NUMBER;i++) { counters[i].resize(0); }
  for(uint32_t i=0; i<RATIOS_NUMBER;i++) { ratios[i].resize(0); }
}

void
Statistics::AppTxCallback (std::string path, Ptr<const Packet> packet)
{
  counters[app_bytesTx][this->size()-1] += packet->GetSize ();
  counters[app_packetsTx][this->size()-1] ++;

  delay_jitter.PrepareTx(packet);  // Tag current time on packet to compute delay and jitter on reception
}

void
Statistics::AppRxCallback (std::string path, Ptr<const Packet> packet, const Address &from)
{
  counters[app_bytesRx][this->size()-1] += packet->GetSize ();
  counters[app_packetsRx][this->size()-1] ++;

  delay_jitter.RecordRx(packet);  // Read Time Tag on packet to compute delay and jitter
  totalDelay += delay_jitter.GetLastDelay().GetMilliSeconds();

  // Get the Node ID Tag to identify the packet sender
  NodeIDTag nodeID;
  packet->PeekPacketTag(nodeID);
  uint32_t senderID = nodeID.GetSimpleValue();

  if(pktCounterPerSender.find(senderID)!=pktCounterPerSender.end())
    {
      pktCounterPerSender[senderID] = (pktCounterPerSender[senderID])+1;
    }
  else
    {
      pktCounterPerSender.insert(std::make_pair(senderID, 1));
    }
}

void
Statistics::PhyNormalCollisionCallback(std::string context, Ptr<const Packet> packet)
{
  counters[phy_nCollision][this->size()-1] ++;
}

void
Statistics::PhyLateCollisionCallback(std::string context, Ptr<const Packet> packet)
{
  counters[phy_lCollision][this->size()-1] ++;
}

void
Statistics::PhyRxErrorCallback (std::string context, Ptr<const Packet> packet, double snr)
{
  counters[phy_RxErr][this->size()-1] ++;
}

void
Statistics::PhyRxOkCallback(std::string context, Ptr<const Packet> packet, double snr, WifiMode mode, enum WifiPreamble preamble)
{
  counters[phy_RxOk][this->size()-1] ++;
}

void
Statistics::MacTxOkCallback(std::string context, const WifiMacHeader &header)
{
  counters[mac_TxOk][this->size()-1] ++;
}

void
Statistics::MacTxErrCallback(std::string context, const WifiMacHeader &header)
{
  counters[mac_TxErr][this->size()-1] ++;
}

void
Statistics::MacTxDataFailedCallback(std::string context, Mac48Address address)
{
  counters[mac_TxFail][this->size()-1] ++;
}

void
Statistics::MacTxFinalDataFailedCallback(std::string context, Mac48Address address)
{
  counters[mac_TxErr][this->size()-1] ++;
}

void
Statistics::MacQueueFullDropCallback(std::string context, Ptr<const Packet> packet)
{
  counters[mac_FullQueueDrop][this->size()-1] ++;
}

void
Statistics::MacQueueTimeoutDropCallback(std::string context, Ptr<const Packet> packet)
{
  counters[mac_PacketTimeout][this->size()-1] ++;
}

void
Statistics::CalcSummary(void)
{
  for(uint32_t i=0; i < COUNTERS_NUMBER; i++)
    {
      // Compute Total
      uint64_t sum = 0;
      for(uint32_t j=0; j < this->size()-1; j++) { sum += counters[i][j]; }
      summaryC[total][i] = sum;

      // Compute Mean
      summaryC[mean][i] = this->size()>0 ? summaryC[total][i] / (double)this->size() : 0;

      // Compute Standard Deviation
      summaryC[stdDeviation][i] = 0;
      for(uint32_t j=0; j < this->size(); j++)
	{
	  summaryC[stdDeviation][i] += std::pow((this->counters[i][j] - summaryC[mean][i]), 2.0); // Compute Variance
	}
      summaryC[stdDeviation][i] = std::sqrt( (summaryC[stdDeviation][i] / this->size()) );	// and Standard Deviation

      // Compute Coefficient of Variation
      summaryC[cov][i] = summaryC[stdDeviation][i] / summaryC[mean][i];

      // Confidence Interval 95%
      double Z = 1.96;	// From normal distribution table
      summaryC[confidenceInterval95][i] = (Z * summaryC[stdDeviation][i] / std::sqrt(this->size()));

      // Confidence Interval 99%
      Z = 2.575;	// From normal distribution table
      summaryC[confidenceInterval99][i] = (Z * summaryC[stdDeviation][i] / std::sqrt(this->size()));

      // Compute Median
      std::vector<uint32_t> scores;
      for(uint32_t j=0; j < this->size(); j++) scores.push_back(this->counters[i][j]);
      sort(scores.begin(), scores.end());	// Order the samples and
      size_t size = scores.size();		// Get the median
      summaryC[median][i] = (size%2==0) ? (scores[size/2-1]+scores[size/2])/2 : scores[size / 2];
    }

  for(uint32_t i=0; i < RATIOS_NUMBER; i++)
    {
      // Compute Total
      double sum = 0;
      for(uint32_t j=0; j < this->size()-1; j++) { sum += ratios[i][j]; }
      summaryR[total][i] = sum;

      // Compute Mean
      summaryR[mean][i] = this->size()>0 ? summaryR[total][i] / (double)this->size() : 0;

      // Compute Standard Deviation
      summaryR[stdDeviation][i] = 0;
      for(uint32_t j=0; j < this->size(); j++)
	{
	  summaryR[stdDeviation][i] += std::pow((this->ratios[i][j] - summaryR[mean][i]), 2.0); // Compute Variance
	}
      summaryR[stdDeviation][i] = std::sqrt( (summaryR[stdDeviation][i] / this->size()) );	// and Standard Deviation

      // Compute Coefficient of Variation
      summaryR[cov][i] = summaryR[stdDeviation][i] / summaryR[mean][i];

      // Confidence Interval 95%
      double Z = 1.96;	// From normal distribution table
      //summaryR[confidenceInterval95][i] = (Z * std::sqrt((summaryR[mean][i]*(1-summaryR[mean][i])/this->size())));
      summaryR[confidenceInterval95][i] = (Z * summaryR[stdDeviation][i] / std::sqrt(this->size()));


      // Confidence Interval 99%
      Z = 2.575;	// From normal distribution table
      summaryR[confidenceInterval99][i] = (Z * std::sqrt((summaryR[mean][i]*(1-summaryR[mean][i])/this->size())));

      // Compute Median
      std::vector<uint32_t> scores;
      for(uint32_t j=0; j < this->size(); j++) scores.push_back(this->ratios[i][j]);
      sort(scores.begin(), scores.end());	// Order the samples and
      size_t size = scores.size();		// Get the median
      summaryR[median][i] = (size%2==0) ? (scores[size/2-1]+scores[size/2])/2 : scores[size / 2];
    }

  // Throughput and Delay are means (not proportions)
  summaryR[confidenceInterval95][throughput] = 1.96 * summaryR[stdDeviation][throughput] / std::sqrt(this->size());
  summaryR[confidenceInterval99][throughput] = 2.575 * summaryR[stdDeviation][throughput] / std::sqrt(this->size());
  summaryR[confidenceInterval95][meanDelay] = 1.96 * summaryR[stdDeviation][meanDelay] / std::sqrt(this->size());
  summaryR[confidenceInterval99][meanDelay] = 2.575 * summaryR[stdDeviation][meanDelay] / std::sqrt(this->size());

}

void
Statistics::CalcRatios(void)
{
  uint32_t p = this->size()-1;

  if(counters[app_packetsTx][p]==0) return;

  // Compute summations
  uint32_t totalTx = counters[mac_TxFail][p] + counters[mac_TxOk][p];
  double totalRxOk = counters[phy_RxOk][p]/2.0; // Not count ACK in RX OK

  // Compute ratios
  if(totalRxOk>0)
    {
      // First packet counts in rxError and subsequent in collision counters
      uint32_t totalCollisions = counters[phy_RxErr][p] + counters[phy_nCollision][p] + counters[phy_lCollision][p];
      ratios[collision][p] = totalCollisions / (double)(totalCollisions + totalRxOk);

      // PHY Rx Errors counts only first packet in a collision
      ratios[phyRxErr][p] = counters[phy_RxErr][p] / (counters[phy_RxErr][p] + totalRxOk);
    }

  ratios[macTxFail][p] = (totalTx>0) ? counters[mac_TxFail][p] / (double)totalTx : 0;
  ratios[txPktLoss][p] = counters[mac_TxErr][p] / (double)(counters[mac_TxErr][p] + counters[mac_TxOk][p]);

  ratios[genPktLoss][p] = ( counters[app_packetsTx][p] - counters[app_packetsRx][p] ) / (double) counters[app_packetsTx][p];
  ratios[macQueueFullDrop][p] = counters[mac_FullQueueDrop][p] / (double) counters[app_packetsTx][p];

  uint32_t totalQueued = counters[app_packetsTx][p] - counters[mac_FullQueueDrop][p];
  ratios[macPktTimeout][p] = counters[mac_PacketTimeout][p] / (double)totalQueued ;
  ratios[macPktLoss][p] = ( counters[mac_TxErr][p] + counters[mac_PacketTimeout][p]) / (double)totalQueued ;

  ratios[meanDelay][p] = totalRxOk > 0 ? totalDelay / totalRxOk : 0;

  // Calculate Fairness Index = (sum(xi))^2 / n*sum(xi^2)
  uint64_t x1, x2; x1=0; x2=0;
  uint32_t nodes = 0;
  std::map<uint32_t, uint32_t>::iterator it = pktCounterPerSender.begin();
  while(it != pktCounterPerSender.end())
    {
      nodes++;
      x1+=it->second;
      x2+=(it->second*it->second);
      it++;
    }
  ratios[fairnessIdx][p] = (x1*x1)/(double)(nodes*x2);
}

void
Statistics::PlotStatistics (double x)
{
  // Add sample to Graphs of Counters
  for(uint32_t i=0; i < COUNTERS_NUMBER; i++)
    {
      m_output_counter[i].Add(x, summaryC[mean][i], summaryC[confidenceInterval95][i]);
    }

  // Add sample to Graphs of Ratios
  for(uint32_t i=0; i < RATIOS_NUMBER; i++)
    {
      m_output_ratio[i].Add(x, summaryR[mean][i], summaryR[confidenceInterval95][i]);
    }


}

void
Statistics::SaveStatistics (double x, uint32_t seconds, const std::string filename)
{
  this->CalcRatios();	// Calculate ratios in the last sample

  // Adjust throughput accordingly time interval
  for(uint32_t i = 0; i < this->size(); i++)
    {
      ratios[throughput][i] = ((counters[app_bytesRx][i] * 8.0) / (1000000 * seconds));
    }

  this->CalcSummary();		// Compute Summary statistics
  this->PlotStatistics(x);	// Put data and summary in Graph data set
  this->ExportCSV(filename);	// Export data and statistics to CSV file
  this->ResetStatistics();	// Prepare a new sample
}

void
Statistics::ImportStatistics (double x, uint32_t seconds, std::vector<uint64_t> c[COUNTERS_NUMBER], std::vector<double> r[RATIOS_NUMBER])
{
  this->ResetStatistics();

  // Place counters/ratios from parameters on its place into the class
  for(uint32_t i=0; i < COUNTERS_NUMBER; i++)
    {
      counters[i].resize(c->size());
      for(uint32_t j=0; j < c->size(); j++)
        {
          counters[i][j] = c[i][j];
        }
    }

  for(uint32_t i=0; i < RATIOS_NUMBER; i++)
    {
      ratios[i].resize(r->size());
      for(uint32_t j=0; j < r->size(); j++)
        {
          ratios[i][j] = r[i][j];
        }
    }

  // Compute Summaries
  this->CalcSummary();
  this->PlotStatistics(x);
}

void
Statistics::ExportCSV(std::string filename)
{
  filename = filename + ".csv";

  FILE *fp;
  fp = fopen(filename.c_str(), "w+");

  // Create Counters Header line in CSV file
  std::string header = "sample";
  for(uint32_t i=0; i < COUNTERS_NUMBER; i++) { header = header + "," + counterName[i]; }
  fprintf(fp, "%s", header.c_str());

  // Add one line of counters for each sample
  for(uint32_t i=0; i < this->size(); i++)
    {
      fprintf(fp, "\n%u",i+1);
      for(uint32_t j=0; j < COUNTERS_NUMBER; j++)
	{
	  fprintf(fp,",%lu", this->counters[j][i]);	// Use this to x32
	  // fprintf(fp,",%llu", this->counters[j][i]);  // Use this to x64
	}
    }

  // Export the summary statistics of counters to CSV
  for(uint32_t i=0; i < SUMMARIES_NUMBER; i++)
	{
	  fprintf(fp,"\n%s", summary[i].c_str());
	  for(uint32_t j=0; j < COUNTERS_NUMBER; j++)
	    {
	      fprintf(fp,",%f", summaryC[i][j]);
 	    }
	}

  // Create Ratios Header line in CSV file
  header = "ratio";
  for(uint32_t i=0; i < RATIOS_NUMBER; i++) { header = header + "," + ratioName[i]; }
  fprintf(fp, "\n\n%s,", header.c_str());

  // Add one line for each sample
  for(uint32_t i=0; i < this->size(); i++)
    {
      fprintf(fp, "\n%u",i+1);
      for(uint32_t j=0; j < RATIOS_NUMBER; j++)
	{
	  fprintf(fp,",%f", this->ratios[j][i]);
	}
    }

  // Export the summary statistics of ratios to CSV
  for(uint32_t i=0; i < SUMMARIES_NUMBER; i++)
	{
	  fprintf(fp,"\n%s", summary[i].c_str());
	  for(uint32_t j=0; j < RATIOS_NUMBER; j++)
	    {
	      fprintf(fp,",%f", summaryR[i][j]);
 	    }
	}

  fprintf(fp, "\n");
  fclose(fp);
  NS_LOG_INFO("Statistics saved to " << filename);
}

Gnuplot2dDataset
Statistics::GetCounterDatafile(counter sample)
{
  return m_output_counter[sample];
}

Gnuplot2dDataset
Statistics::GetCounterDatafile(uint8_t sample)
{
  return m_output_counter[sample];
}

Gnuplot2dDataset
Statistics::GetRatioDatafile(ratio sample)
{
  return m_output_ratio[sample];
}

Gnuplot2dDataset
Statistics::GetRatioDatafile(uint8_t sample)
{
  return m_output_ratio[sample];
}

//------------------------------------------------------------------------
// End class NodeStatistics
//------------------------------------------------------------------------

//------------------------------------------------------------------------
// Helper Functions
//------------------------------------------------------------------------
NetDeviceContainer
CreateWifiDevices(Ptr<YansWifiChannel> channel, const WifiMacHelper &macHelper, enum WifiPhyStandard standard, NodeContainer nodes, uint8_t numAntennas = 1)
{
  NetDeviceContainer devices;
  for (NodeContainer::Iterator i = nodes.Begin (); i != nodes.End (); ++i)
    {
      Ptr<Node> node = *i;
      Ptr<WifiNetDevice> device = CreateObject<WifiNetDevice> ();

      Ptr<CollissionDetectorWifiPhy> phy = CreateObject<CollissionDetectorWifiPhy>();
      Ptr<ErrorRateModel> error = CreateObject<YansErrorRateModel> ();
      phy->SetErrorRateModel (error);
      phy->SetChannel (channel);
      //phy->SetMobility(node);
      phy->SetDevice(device);
      phy->ConfigureStandard (standard);
      phy->SetNumberOfReceiveAntennas(numAntennas);
      phy->SetNumberOfTransmitAntennas(numAntennas);

      Ptr<ConstantRateWifiManager> manager = CreateObject<ConstantRateWifiManager> ();
      manager->SetAttribute("DataMode", StringValue("HtMcs7"));  	//MCS 7 - 64-QAM 5/6 - 6.5 mbps
      manager->SetAttribute("ControlMode", StringValue("HtMcs0"));	//MCS 0 - BPSK   1/2 -  65 mbps
//      manager->SetAttribute("DataMode", StringValue("ErpOfdmRate54Mbps"));
//      manager->SetAttribute("AckMode", StringValue("ErpOfdmRate54Mbps"));
//      manager->SetAttribute("ControlMode", StringValue("ErpOfdmRate54Mbps"));

      manager->SetMaxSsrc(7); // Max 7 retries in RTS
      manager->SetMaxSlrc(7); // Max 7 retries in Long Retries Counter (greater than RTS threshold)
      manager->SetMaxSsrc(7); // Max 7 retries in Short Retries Counter (lower than RTS threshold)

      Ptr<WifiMac> mac = macHelper.Create ();
      mac->SetAddress (Mac48Address::Allocate ());
      mac->ConfigureStandard (standard);

      device->SetMac (mac);
      device->SetPhy (phy);
      device->SetRemoteStationManager (manager);
      node->AddDevice (device);
      devices.Add (device);
    }
  return devices;
}

/*
 * Set Tag ID to classify packet in EDCA categories
 */
void
TagMarker (uint8_t acIndex, uint32_t NodeID, Ptr<const Packet> packet)
{
  NodeIDTag idTag;		// Tag to identify sender in fairness index
  idTag.SetSimpleValue(NodeID); // Use node ID to identify sender
  packet->AddPacketTag(idTag);	// on each packet

  uint8_t tid;
  switch(acIndex)
  {
    case AC_VO: tid = 7; break;
    case AC_VI: tid = 5; break;
    case AC_BE: tid = 3; break;
    case AC_BK: tid = 2; break;
    default: tid = 0;
  }
 QosTag qosTag;			// Tag to identify access category
 qosTag.SetTid(tid);		// Set accordingly traffic type
 packet->AddPacketTag (qosTag); // on each packet
}

/*
 * Set Enlarged Initial Contention Window to maintain same duration with reduced slots
 */
void
SetCWsize(Ptr<WifiMac> macdev)
{
	PointerValue ptr;
	macdev->GetAttribute("DcaTxop", ptr);
	Ptr<DcaTxop> dca = ptr.Get<DcaTxop>();
	dca->SetMinCw(31);
	dca->SetMaxCw(1023);
	dca->SetAifsn(4);

	// double each CW
	for(uint8_t i=0; i<4; i++)
	{
		std::string ac;
		uint32_t cwmin, cwmax, aifsn;
		switch(i)
		{
		case 0: ac = "VO_EdcaTxopN"; aifsn = 4; cwmin = 7; cwmax = 15; break;
		case 1: ac = "VI_EdcaTxopN"; aifsn = 4; cwmin = 15; cwmax = 31; break;
		case 2: ac = "BE_EdcaTxopN"; aifsn = 6; cwmin = 31; cwmax = 1023; break;
		case 3: ac = "BK_EdcaTxopN"; aifsn = 14; cwmin = 31; cwmax = 1023; break;
		}
		Ptr<EdcaTxopN> edca;
		macdev->GetAttribute(ac, ptr);
		edca = ptr.Get<EdcaTxopN>();
		edca->SetMinCw(cwmin);
		edca->SetMaxCw(cwmax);
		edca->SetAifsn(aifsn); // AIFS is based on Slot Time
		//edca->m_LimitLegacySlots = (i>1); // Use new slots only for VOice or VIdeo traffic
	}
}

void
SetEICW(NetDeviceContainer devices)
{
  for(NetDeviceContainer::Iterator i = devices.Begin(); i != devices.End(); i++)
    {
      Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice>(*i);
      Ptr<WifiMac> macdev = dev->GetMac();

      macdev->SetSlot(macdev->GetSlot()/2); //Set SlotTime in half of normal duration

      SetCWsize(macdev);	// Set Contention Windows size and AIFS
    }
}

/*
 * Resset Contention Window to default values
 */
void
RessetCWsize(Ptr<WifiMac> macdev)
{
	PointerValue ptr;
	macdev->GetAttribute("DcaTxop", ptr);
	Ptr<DcaTxop> dca = ptr.Get<DcaTxop>();
	dca->SetMinCw(15);
	dca->SetMaxCw(1023);
	dca->SetAifsn(2);

	// double each CW
	for(uint8_t i=0; i<4; i++)
	{
		std::string ac;
		uint32_t cwmin, cwmax, aifsn;
		switch(i)
		{
		case 0: ac = "VO_EdcaTxopN"; aifsn = 2; cwmin = 3; cwmax = 7; break;
		case 1: ac = "VI_EdcaTxopN"; aifsn = 2; cwmin = 7; cwmax = 15; break;
		case 2: ac = "BE_EdcaTxopN"; aifsn = 3; cwmin = 15; cwmax = 1023; break;
		case 3: ac = "BK_EdcaTxopN"; aifsn = 7; cwmin = 15; cwmax = 1023; break;
		}
		Ptr<EdcaTxopN> edca;
		macdev->GetAttribute(ac, ptr);
		edca = ptr.Get<EdcaTxopN>();
		edca->SetMinCw(cwmin);
		edca->SetMaxCw(cwmax);
		edca->SetAifsn(aifsn); // AIFS is based on Slot Time
		//edca->m_LimitLegacySlots = (i>1); // Use new slots only for VOice or VIdeo traffic
	}
}

/*
 * Set TDuCSMA Contention Window to prioritize device
 */
void
SetTDuCWsize(Ptr<WifiMac> macdev)
{
	PointerValue ptr;
	macdev->GetAttribute("DcaTxop", ptr);
	Ptr<DcaTxop> dca = ptr.Get<DcaTxop>();
	dca->SetMinCw(1);
	dca->SetMaxCw(1);
	dca->SetAifsn(2);

	// reduce each CW to minimum size
	for(uint8_t i=0; i<4; i++)
	{
		std::string ac;
		switch(i)
		{
		case 0: ac = "VO_EdcaTxopN"; break;
		case 1: ac = "VI_EdcaTxopN"; break;
		case 2: ac = "BE_EdcaTxopN"; break;
		case 3: ac = "BK_EdcaTxopN"; break;
		}
		Ptr<EdcaTxopN> edca;
		macdev->GetAttribute(ac, ptr);
		edca = ptr.Get<EdcaTxopN>();
		edca->SetMinCw(1);
		edca->SetMaxCw(1);
		edca->SetAifsn(2);
	}
}

/*
 * Generate an UDP/TCP flow
 */
void
GenerateTraffic(NodeContainer servers, NodeContainer clients,
		   Time start, double interval, bool UDP, double cbr_kbps,
		   bool uplink = false, uint32_t size = 256, uint8_t ac = AC_BE, uint16_t port = 50000, uint32_t maxBytes = 0)
{
  std::string protocol = UDP ? "ns3::UdpSocketFactory" : "ns3::TcpSocketFactory";
  std::ostringstream dataRate;
  dataRate<<cbr_kbps<<"kbps";
  Time stop = start + Seconds(interval);

  // Create UDP flows between clients and servers
  for(NodeContainer::Iterator server = servers.Begin(); server != servers.End(); server++)
    {
      for(NodeContainer::Iterator client = clients.Begin(); client != clients.End(); client++)
	{
	  Ptr<Ipv4> destAddress;  // Get node to receive packets
	  if(uplink) { destAddress = ((*server)->GetObject<Ipv4>()); }
	  else	     { destAddress = ((*client)->GetObject<Ipv4>()); }
	  Ipv4Address remoteAddress (destAddress->GetAddress (1,0).GetLocal ());
	  InetSocketAddress remote = InetSocketAddress (remoteAddress, port);

	  // Get sender ID data
	  Ptr<Node> sender; // Get node to sent packets
	  sender = uplink ? (*client) : (*server);
//	  uint32_t nodeID = sender->GetId();

	  // Set application parameters
	  Ptr<OnOffApplication> app = CreateObject<OnOffApplication> ();
	  app->SetAttribute("Protocol", StringValue (protocol));
	  app->SetAttribute("Remote", AddressValue (remote));
	  app->SetAttribute("DataRate", DataRateValue (DataRate(dataRate.str().c_str())));
	  app->SetAttribute("PacketSize", UintegerValue (size));
	  app->SetAttribute("OnTime", StringValue ("ns3::ParetoRandomVariable[Mean=0.05][Shape=1.5]"));
	  app->SetAttribute("OffTime", StringValue ("ns3::ParetoRandomVariable[Mean=0.05][Shape=1.5]"));
	  app->SetAttribute ("MaxBytes", UintegerValue(maxBytes));
//	  app->TraceConnectWithoutContext("Tx",  MakeBoundCallback (&TagMarker, ac, nodeID));
	  app->SetStartTime(start);
	  app->SetStopTime(stop);

	  // Install application on sender
	  sender->AddApplication(app);
	}
    }
}

void
ConnectAPPCallbacks(NodeContainer WifiNodes, Statistics * statistics)
{
  for(NodeContainer::Iterator i = WifiNodes.Begin(); i != WifiNodes.End(); i++)
    {
      uint32_t nodeID = (*i)->GetId();

      std::ostringstream paramRx; // Callback Trace to Collect Received packets in PacketSink
      paramRx<<"/NodeList/"<<(nodeID)<<"/ApplicationList/*/$ns3::PacketSink/Rx";
      Config::Connect (paramRx.str().c_str(), MakeCallback (&Statistics::AppRxCallback, statistics));

      std::ostringstream paramTx; // Callback Trace to Collect generated packets in OnOffApplication
      paramTx<<"/NodeList/"<<(nodeID)<<"/ApplicationList/*/$ns3::OnOffApplication/Tx";
      Config::Connect (paramTx.str().c_str(), MakeCallback (&Statistics::AppTxCallback, statistics));
    }
}

void
ConnectPHYCallbacks(NodeContainer WifiNodes, Statistics * statistics)
{
  for(NodeContainer::Iterator i = WifiNodes.Begin(); i != WifiNodes.End(); i++)
    {
      uint32_t nodeID = (*i)->GetId();

      std::ostringstream paramE1;  // Callback Trace to Normal Collisions (at beginning of frame)
      paramE1<<"/NodeList/"<<(nodeID)<<"/DeviceList/*/Phy/PhyNormalCollision";
      Config::Connect (paramE1.str().c_str(), MakeCallback (&Statistics::PhyNormalCollisionCallback, statistics));

      std::ostringstream paramE2;  // Callback Trace to Late Collisions
      paramE2<<"/NodeList/"<<(nodeID)<<"/DeviceList/*/Phy/PhyLateCollision";
      Config::Connect (paramE2.str().c_str(), MakeCallback (&Statistics::PhyLateCollisionCallback, statistics));

      std::ostringstream paramE3;  // Callback Trace to Collect Rx Errors at PHY State Helper
      paramE3<<"/NodeList/"<<(nodeID)<<"/DeviceList/*/Phy/State/RxError";
      Config::Connect (paramE3.str().c_str(), MakeCallback (&Statistics::PhyRxErrorCallback, statistics));

      std::ostringstream paramE4;  // Callback Trace to Collect Rx OK at PHY State Helper
      paramE4<<"/NodeList/"<<(nodeID)<<"/DeviceList/*/Phy/State/RxOk";
      Config::Connect (paramE4.str().c_str(), MakeCallback (&Statistics::PhyRxOkCallback, statistics));
    }
}

void
ConnectMACCallbacks(NodeContainer WifiNodes, Statistics * statistics)
{
  for(NodeContainer::Iterator i = WifiNodes.Begin(); i != WifiNodes.End(); i++)
    {
      uint32_t nodeID = (*i)->GetId();

      std::ostringstream paramE5;  // Callback Trace to Collect Tx OK at MAC layer (Got ACK)
      paramE5<<"/NodeList/"<<(nodeID)<<"/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/TxOkHeader";
      Config::Connect (paramE5.str().c_str(), MakeCallback (&Statistics::MacTxOkCallback, statistics));

      std::ostringstream paramE6;  // Callback Trace to Collect Tx Failed at MAC layer (Missed ACK)
      paramE6<<"/NodeList/"<<(nodeID)<<"/DeviceList/*/RemoteStationManager/MacTxDataFailed";
      Config::Connect (paramE6.str().c_str(), MakeCallback (&Statistics::MacTxDataFailedCallback, statistics));

      std::ostringstream paramE7a;  // Callback Trace to Collect Tx Final Failed at MAC layer (Max Retries Reached)
      paramE7a<<"/NodeList/"<<(nodeID)<<"/DeviceList/*/RemoteStationManager/MacTxFinalDataFailed";
      Config::Connect (paramE7a.str().c_str(), MakeCallback (&Statistics::MacTxFinalDataFailedCallback, statistics));

      std::ostringstream paramE7;  // Callback Trace to Collect Tx Err at MAC layer (Drop after all retries)
      paramE7<<"/NodeList/"<<(nodeID)<<"DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/TxErrHeader";
      Config::Connect (paramE7.str().c_str(), MakeCallback (&Statistics::MacTxErrCallback, statistics));

      const std::string queueName[5] = { "DcaTxop", "VO_EdcaTxopN", "VI_EdcaTxopN", "BE_EdcaTxopN", "BK_EdcaTxopN"};
      for(uint8_t j=0; j<5; j++)
	{
	  std::string queueType = j==0 ? "DcaTxop" : "EdcaTxopN";
	  std::ostringstream paramE8;  // Callback Trace to Collect MAC Drop when Queue is full
	  paramE8<<"/NodeList/"<<(nodeID)<<"/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/"<<queueName[j]<<"/$ns3::"<<queueType<<"/Queue/$ns3::WifiMacQueue/FullMacQueueDrop";
	  Config::Connect (paramE8.str().c_str(), MakeCallback (&Statistics::MacQueueFullDropCallback, statistics));
	  std::ostringstream paramE9;  // Callback Trace to Collect MAC Drop when packet is timeouted
	  paramE9<<"/NodeList/"<<(nodeID)<<"/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/"<<queueName[j]<<"/$ns3::"<<queueType<<"/Queue/$ns3::WifiMacQueue/PacketTimeoutDrop";
	  Config::Connect (paramE9.str().c_str(), MakeCallback (&Statistics::MacQueueTimeoutDropCallback, statistics));
	}
    }
}

void
ConnectCallbacks(NodeContainer WifiNodes, Statistics * statistics)
{
  ConnectPHYCallbacks(WifiNodes, statistics);
  ConnectMACCallbacks(WifiNodes, statistics);
  ConnectAPPCallbacks(WifiNodes, statistics);
}

//------------------------------------------------------------------------
// End Helper Functions
//------------------------------------------------------------------------

//------------------------------------------------------------------------
// CSMA/ECA Functions
//------------------------------------------------------------------------

static void
SetupECA(NodeContainer *wifiNodes)
{
	/* Resetting node's state and stats */
	for(uint32_t i = 0; i < wifiNodes->GetN (); i++){
		for(uint32_t j = 0; j < wifiNodes->GetN (); j++){
			if(i == j)
				continue;

			/* Configuring the ARP entries to avoid control traffic */
			Address mac = wifiNodes->Get (j)->GetDevice (0)->GetAddress ();
			/* Ipv4 is the forwarding table class */
			Ipv4Address ip = wifiNodes->Get (j)->GetObject<Ipv4> ()->GetAddress (1,0).GetLocal ();
			Ptr<ArpCache> arpCache = wifiNodes->Get (i)->GetObject<Ipv4L3Protocol> ()->GetInterface (1)->GetArpCache ();

			if (arpCache == NULL)
				arpCache = CreateObject<ArpCache>( );
			arpCache->SetAliveTimeout (Seconds (3600 * 24 * 365));
			ArpCache::Entry *entry = arpCache->Add (ip);
			entry->MarkWaitReply(0);
			entry->MarkAlive(mac);
		}

		Ptr<WifiMac> wifiMac = wifiNodes->Get (i)->GetDevice (0)->GetObject<WifiNetDevice> ()->GetMac ();

		Ptr<DcfManager> dcfManager = wifiNodes->Get (i)->GetDevice (0)->GetObject<WifiNetDevice> ()
	        				->GetMac ()->GetObject<RegularWifiMac> ()->GetDcfManager ();

		Ptr<DcaTxop> dca = wifiNodes->Get (i)->GetDevice (0)->GetObject<WifiNetDevice> ()
	        				->GetMac ()->GetObject<RegularWifiMac> ()->GetDcaTxop ();

		dca->ResetStats ();
		dcfManager->SetEnvironmentForECA (true, false, 0, false);
		//		  dcfManager->SetEnvironmentForECA (config.hysteresis, config.bitmap,
		//				  config.stickiness, config.dynStick);

		//		  if (config.CWmin > 0)
		//			  dca->SetMinCw(config.CWmin);

		/* Setting the Ack timeout and EIFS no DIFS to be equal to DIFS */
		//		  if (config.EIFSnoDIFS != 314)
		//			  wifiMac->SetEifsNoDifs (MicroSeconds (config.EIFSnoDIFS));
		//		  if (config.ackTimeout != 340)
		//			  wifiMac->SetAckTimeout (MicroSeconds (config.ackTimeout));


		/* Setting all the nasty stuff for Schedule Reset */
		//		  if (config.bitmap == true)
		//		  {
		//			  if (config.srActivationThreshold == 1)
		//				  dca->SetScheduleResetActivationThreshold (config.srActivationThreshold);
		//			  if (config.srConservative)
		//				  dca->SetScheduleConservative ();
		//			  if (config.srResetMode)
		//				  dca->SetScheduleResetMode (); //Halving or reset?
		//		  }

	}
}

//------------------------------------------------------------------------
// End CSMA/ECA Functions
//------------------------------------------------------------------------

//------------------------------------------------------------------------
// TDuCSMA Functions
//------------------------------------------------------------------------

void
SetupTDuCSMA(uint32_t ativo, NodeContainer STAnodes, double interval)
{
	uint32_t ativar = (ativo<STAnodes.GetN()) ? ativo + 1 : 0;

//	std::cout << "TDuCSMA - Desligando estacao " << ativo << " e ligando estacao " << ativar << " em " << Simulator::Now().GetSeconds() << std::endl;

	// Disable current priority device
	if(ativo<STAnodes.GetN())
	{
		Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice>(STAnodes.Get(ativo)->GetDevice(0));
	    Ptr<WifiMac> macdev = dev->GetMac();

	    if(macdev->GetSlot().GetMicroSeconds()<5.0)
	    {
	    	SetCWsize(macdev);		// if reduced slot size
	    } else {
	    	RessetCWsize(macdev);	// if normal slot size
	    }
	}

	// Enable next priority device
    if(ativar<STAnodes.GetN())
    {
    	Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice>(STAnodes.Get(ativar)->GetDevice(0));
    	Ptr<WifiMac> macdev = dev->GetMac();
        SetTDuCWsize(macdev);
    }

	// Agenda a próxima troca de estacao prioritaria
	Simulator::Schedule(Seconds(interval), &SetupTDuCSMA, ativar, STAnodes, interval);
}

//------------------------------------------------------------------------
// End TDuCSMA Functions
//------------------------------------------------------------------------

//------------------------------------------------------------------------
// File System Functions
//------------------------------------------------------------------------

struct parameters
{
  uint16_t packetSize;
  uint16_t sampleInterval;
  uint8_t  repetitions;
  uint8_t  numBSS;
  uint8_t  numSTA;
  uint8_t  slotTime;
  uint8_t  ac;
  bool	   RTSon;
  bool	   legacy;
  bool	   ECA;
  bool 	   TDuCSMA;
  char 	   scenario;
};

std::string
SlotTime(parameters p)
{
  std::string st = "";
  if(p.legacy)
    switch(p.slotTime)
      {
	case 9: st = "20us"; break;
	case 4: st = "10us"; break;
	case 0: st = "20+10us"; break;
      }
  else
    switch(p.slotTime)
      {
	case 9: st = "9us"; break;
	case 4: st = "4.5us"; break;
	case 0: st = "9+4.5us"; break;
      }
  return st;
}

std::string
AccessCategory(parameters p)
{
  std::string legend = "";
  switch(p.ac)
  {
    case 3: legend = "AC VO"; break;
    case 2: legend = "AC VI"; break;
    case 0: legend = "AC BE"; break;
    case 1: legend = "mixed"; break;
  }
  return legend;
}

const std::string
Filename(parameters p)
{
  const std::string prefix = "CI927";
  std::string slotTime = SlotTime(p);
  if(slotTime != "") slotTime = "-" + slotTime ;

  std::string type;
  switch(p.scenario)
  {
    case 'A': type = "sat"; break;
    case 'B': type = "nsat54"; break;
    case 'C': type = "mixA"; break;
    case 'D': type = "mixB"; break;
    case 'E':
    	if( p.slotTime == 9 && !p.ECA && !p.TDuCSMA) type = "NORMAL";
    	if( p.slotTime == 4 && !p.ECA && !p.TDuCSMA) type = "EICW";
    	if( p.slotTime == 9 &&  p.ECA && !p.TDuCSMA) type = "ECA";
    	if(	p.slotTime == 9 && !p.ECA &&  p.TDuCSMA) type = "TDuCSMA";
    	if( p.slotTime == 4 &&  p.ECA && !p.TDuCSMA) type = "EICW+ECA";
    	if(	p.slotTime == 9 &&  p.ECA &&  p.TDuCSMA) type = "ECA+TDu";
    	if(	p.slotTime == 4 && !p.ECA &&  p.TDuCSMA) type = "EICW+TDu";
    	if(	p.slotTime == 4 &&  p.ECA &&  p.TDuCSMA) type = "EICW+ECA+TDu"; break;
    	type = "error"; break;
  }

  std::string trafficType = "";
  switch(p.ac)
  {
    case AC_VO: trafficType = "-VO"; break;
    case AC_VI: trafficType = "-VI"; break;
    case AC_BE: trafficType = "-BE"; break;
    case 1: trafficType = "-MX"; break;
  }

  if(p.scenario == 'E' && p.ECA) { trafficType = "-ECA"; }

  std::string stations = "";
  if(p.numSTA>0)
    {
      std::stringstream sta;
      sta << "-" << (uint16_t)p.numSTA << "sta";
      stations = sta.str();
    }

  std::ostringstream id;
  id<<prefix<<"-"<<type<<slotTime<<trafficType<<"-"<<((uint16_t)p.repetitions)<<"x"<<(p.sampleInterval)<<"sec-"<<p.packetSize<<"bpp-"<<((uint16_t)p.numBSS)<<"bss"<<stations;
  return id.str().c_str();
}

bool
CheckFile(parameters p, Statistics * statistics)
{
  std::string filename = Filename(p) + ".csv";

  std::ifstream file(filename.c_str());
  if (file.good())
    {
      NS_LOG_INFO("Reading CSV file " + filename);

      std::string line;
      std::getline(file,line);
      std::stringstream lineStream(line);

      std::vector<std::string> data;
      std::string cell;

      while(std::getline(lineStream,cell,',')) { data.push_back(cell); }

      // Check structure of file (field names)
      if(data.size() != COUNTERS_NUMBER + 1 || data.front() != "sample") return false;
      for(uint32_t i =0 ; i < COUNTERS_NUMBER; i++)
	{
	  if(counterName[i] != data[i+1]) return false;
	}

      std::vector<uint64_t> counters[COUNTERS_NUMBER];
      for(uint32_t i =0 ; i < COUNTERS_NUMBER; i++) counters[i].resize(p.repetitions);

      for(uint32_t i = 0; i < p.repetitions; i++)	// each line of file with counters
	{
	  data.clear();
	  std::getline(file,line);
	  std::stringstream lineStream(line);
	  while(std::getline(lineStream,cell,',')) { data.push_back(cell); }
	  if(data.size() != COUNTERS_NUMBER + 1) return false;
	  for(uint32_t j=0 ; j < COUNTERS_NUMBER; j++)
	    {
	      counters[j][i] = std::atol( data[j+1].c_str() );
	    }
	}

      while(!file.eof())
	{
	  data.clear();
	  std::getline(file,line);
	  std::stringstream lineStream(line);
	  while(std::getline(lineStream,cell,',')) { data.push_back(cell); }
	  if(data.front() == "ratio") break;
	}

      std::vector<double> ratios[RATIOS_NUMBER];
      for(uint32_t i =0 ; i < RATIOS_NUMBER; i++) ratios[i].resize(p.repetitions);

      for(uint32_t i = 0; i < p.repetitions; i++)	// each line of file with counters
	{
	  data.clear();
	  std::getline(file,line);
	  std::stringstream lineStream(line);
	  while(std::getline(lineStream,cell,',')) { data.push_back(cell); }
	  if(data.size() != RATIOS_NUMBER + 1) return false;
	  for(uint32_t j=0 ; j < RATIOS_NUMBER; j++)
	    {
	      ratios[j][i] = std::atof( data[j+1].c_str() );
	    }
	}

      statistics->ImportStatistics(p.numSTA, p.sampleInterval, counters, ratios);

      return true;
    }
  return false;
}

//------------------------------------------------------------------------
// End File System Functions
//------------------------------------------------------------------------

//----------------------------------------------------------------
// GNUPLOT helper functions
//----------------------------------------------------------------

void
FormatLineStyle(std::string filename)
{
  std::string tempfilename = filename + ".tmp";
  const std::string term = "yerrorlines";	// Use "linespoints" when without errorsbars

  uint16_t style = 1;

  std::ifstream in(filename.c_str());
  std::ofstream out(tempfilename.c_str());

  if(in && out)
    {
      std::string line;
      while(std::getline(in, line))
	{
	  size_t curPos = 0;
	  while(true)
	    {
	      size_t pos = line.find(term, curPos);
	      if(pos == std::string::npos) break;
	      std::stringstream newStyle;
	      newStyle << term << " ls " << style;
	      line.replace(pos, 11, newStyle.str());
	      curPos=pos+1; style++;
	    }
	  out << line << '\n';
	}
      in.close();
      out.close();

      std::remove(filename.c_str());
      std::rename(tempfilename.c_str(), filename.c_str());
    }
}

void
plot(std::string scenery, std::vector<Statistics> statistics, std::vector<std::string> legend, uint8_t numSTA, bool generatePDF = true)
{
  NS_LOG_INFO ("Plotting results...");

  for(uint8_t graphType=0; graphType<2; graphType++)
    {
      uint8_t graphCount = graphType==0 ? COUNTERS_NUMBER : RATIOS_NUMBER;

      for(uint8_t graph=0; graph<graphCount; graph++)
        {
          std::string prefix = "graph-";
          std::string outputFileName = "-" + scenery ;
          std::string graphFile = graphType==0 ? counterName[graph] : ratioName[graph];
          std::string unit = graphType==0 ? counterUnits[graph] : ratioUnits[graph];
          std::string title = graphType==0 ? counterLegend[graph] : ratioLegend[graph] + " vs Density";

          Gnuplot gnuplot;
          if(generatePDF)
            {
              std::string pdfConvert = "| epstopdf --filter > ";
              gnuplot = Gnuplot ((pdfConvert + prefix + graphFile + outputFileName + ".pdf").c_str (), graphFile);
            }
          else
            {
              gnuplot = Gnuplot ((prefix + graphFile + outputFileName + ".eps").c_str (), graphFile);
            }

          gnuplot.SetTerminal ("post eps color enhanced font 'Helvetica,18'");
          gnuplot.AppendExtra ("set encoding iso_8859_1");

          gnuplot.SetLegend ("Number of Stations in the Channel", unit);
          gnuplot.SetTitle (title);

          gnuplot.AppendExtra("set key right bottom samplen 6 spacing 1.4 font ',18' reverse Left");
          gnuplot.AppendExtra("");
          if(statistics.size()==3)
            {
              gnuplot.AppendExtra("set style line 1 lt 3 lw 3 pt 5 ps 0 lc 3  # Blue dotted");
              gnuplot.AppendExtra("set style line 2 lt 1 lw 3 pt 5 ps 0 lc 1  # Red line 1");
              gnuplot.AppendExtra("set style line 3 lt 5 lw 3 pt 5 ps 0 lc 7  # Black dot+dash");
            }
          else
            {
              gnuplot.AppendExtra("set style line 1 lt 2 lw 3 pt 7 ps 0 lc 2  # Green dashed");
              gnuplot.AppendExtra("set style line 2 lt 3 lw 3 pt 5 ps 0 lc 3  # Blue dotted");
              gnuplot.AppendExtra("set style line 3 lt 5 lw 3 pt 5 ps 0 lc 7  # Black dot+dash");
              gnuplot.AppendExtra("set style line 4 lt 1 lw 3 pt 5 ps 0 lc 1  # Red line 1");
              gnuplot.AppendExtra("set style line 5 lt 1 lw 3 pt 7 ps 0 lc 3  # Blue line 1");
              gnuplot.AppendExtra("set style line 6 lt 1 lw 3 pt 7 ps 0 lc 7  # Black line 1");
            }

          std::ostringstream xrange, yrange;
          xrange << "set xrange [0:" << ((uint16_t)numSTA) << "]";
          yrange << "set yrange [" << ( graphType && graph<9 ? "0:1" : "*:*" ) << "]";

          gnuplot.AppendExtra("");
          gnuplot.AppendExtra("set tics font ',20'");
          gnuplot.AppendExtra(xrange.str().c_str());
          gnuplot.AppendExtra(yrange.str().c_str());
          gnuplot.AppendExtra("set grid");
          gnuplot.AppendExtra("show grid");
          gnuplot.AppendExtra("set size square");
          gnuplot.AppendExtra("show size");
          gnuplot.AppendExtra("");

          for(uint8_t i=0; i<statistics.size(); i++)
            {
              Gnuplot2dDataset dataTP;

              if(graphType==0) 	{ dataTP = statistics[i].GetCounterDatafile(graph); }
              else		{ dataTP = statistics[i].GetRatioDatafile(graph); }

              dataTP.SetTitle(legend[i]);
              dataTP.SetStyle(Gnuplot2dDataset::LINES_POINTS);
              gnuplot.AddDataset(dataTP);
            }

          std::string filename = prefix + graphFile + outputFileName + ".plt";
          std::ofstream outfile (filename.c_str());
          gnuplot.GenerateOutput(outfile);
          FormatLineStyle(filename);
        }
    }
}

//----------------------------------------------------------------
// End GNUPLOT helper functions
//----------------------------------------------------------------

//------------------------------------------------------------------------
// Experiment
//------------------------------------------------------------------------
void
MySim(Statistics * statistics, parameters p, uint8_t seed, bool traceOn = false, bool animOn = false, bool flowMonitor = false, bool verbose = true)
{
  if(p.numSTA<1 || p.numBSS<1 || p.numBSS>2 || p.ac<0 || p.ac>3) return;

  std::stringstream id;
  id << Filename(p) << "_run" << (uint16_t)(seed+1);
  const std::string version =  id.str();

  // Wait one second to prepare scenery with AP associations etc then starts the simulation
  // Wait one second to empty buffers after simulation
  Time leadUp = Seconds(1.0);
  Time startSim = leadUp;
  Time endSim = startSim + Seconds(p.sampleInterval) + leadUp;

  //---------------------------------------------------------------------------------
  // Setup global parameters
  //---------------------------------------------------------------------------------
  NS_LOG_INFO ("Running simulation... " + version);

  SeedManager::SetSeed(50);	// Fixed Seed and advance run number - See ns3 manual (Seeding and independent replications)
  SeedManager::SetRun(seed);

  // Max MAC Queue size
  Config::SetDefault ("ns3::WifiMacQueue::MaxPacketNumber", UintegerValue (1000));
  Config::SetDefault ("ns3::WifiMacQueue::MaxDelay", TimeValue (MicroSeconds(500*1024)));

  // Enable / Disable RTS CTS
  UintegerValue rtsCtsThreshold = p.RTSon ? UintegerValue (100) : UintegerValue (2200);
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue(rtsCtsThreshold));
  Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue("5000"));

  //---------------------------------------------------------------------------------
  // Setup nodes parameters
  //---------------------------------------------------------------------------------
  if(verbose) NS_LOG_DEBUG ("Setup Nodes...");

  NodeContainer ISPnode; ISPnode.Create(1);
  NodeContainer APSnodes; APSnodes.Create(p.numBSS);
  NodeContainer STAnodes;
  NodeContainer BSSnodes[p.numBSS];
  NetDeviceContainer BSSdevices[p.numBSS];

  //---------------------------------------------------------------------------------
  // Define propagation loss model
  //---------------------------------------------------------------------------------
  if(verbose) NS_LOG_DEBUG ("Setup propagation loss model...");
  Ptr<TwoRayGroundPropagationLossModel> lossModel = CreateObject<TwoRayGroundPropagationLossModel>();
  if(p.legacy) { lossModel->SetFrequency(5.180); } else { lossModel->SetFrequency(2.412); }
  lossModel->SetSystemLoss(1.0);

  //---------------------------------------------------------------------------------
  // Setup WiFi Channel
  //---------------------------------------------------------------------------------
  if(verbose) NS_LOG_DEBUG ("Setup WiFi Channel...");
  Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel>();
  channel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
  channel->SetPropagationLossModel(lossModel);

  //---------------------------------------------------------------------------------
  // Setup PHY and MAC parameters
  //---------------------------------------------------------------------------------
  if(verbose) NS_LOG_DEBUG ("Setup PHY and MAC...");
  HtWifiMacHelper mac = HtWifiMacHelper::Default ();
//  NqosWifiMacHelper mac = NqosWifiMacHelper::Default ();

  for(uint8_t i=0; i < p.numBSS; i++)
    {
      // Configure Access Points
      std::ostringstream bssId;
      bssId<<"BSS-"<<(uint16_t)i;
      Ssid ssid = Ssid (bssId.str().c_str());

      mac.SetType ("ns3::ApWifiMac",
		   "Ssid", SsidValue (ssid),
		   "BeaconGeneration", BooleanValue(true),
		   "EnableBeaconJitter", BooleanValue(true), // Beacon interval must be different on each AP
		   "BeaconInterval", TimeValue(MicroSeconds(100*1024))); // see IEEE Std. 802.11-2012

      // If a legacy device is operating in same channel, all BSS must operate in legacy with 20us slots.
      enum WifiPhyStandard mode = p.legacy ? WIFI_PHY_STANDARD_80211n_2_4GHZ : WIFI_PHY_STANDARD_80211n_5GHZ;
      //enum WifiPhyStandard mode = WIFI_PHY_STANDARD_80211g;

      NetDeviceContainer apDevice = CreateWifiDevices(channel, mac, mode, APSnodes.Get(i), 2);
      BSSnodes[i].Add(APSnodes.Get(i));
      BSSdevices[i].Add(apDevice);

      uint8_t thisBSS = p.numSTA / p.numBSS;
      if(i==0) thisBSS += (p.numSTA % p.numBSS);

      // Configure stations on each Access Point
      mac.SetType ("ns3::StaWifiMac",
		   "Ssid", SsidValue (ssid),
		   "ActiveProbing", BooleanValue (false));

      for(uint8_t j=0; j < thisBSS; j++)
	{
	  NodeContainer node; node.Create(1);
	  // Four stations on each stall (notebook, monitor, smartphone and storage)
	  NetDeviceContainer staDevice = CreateWifiDevices(channel, mac, mode, node.Get(0));
	  STAnodes.Add(node.Get(0));
	  BSSnodes[i].Add(node.Get(0));
	  BSSdevices[i].Add(staDevice);
	}

      // Set Enlarged Contention Window and Slot Time according the proposal (half slot time and double CW)
      if(p.slotTime==4) { SetEICW(BSSdevices[i]); }

      if(p.slotTime==0)  // Mixed mode - half operating with 9us slots and half with 4.5us (or 20us -> 10us if in legacy mode)
	{
	  if(p.scenario=='A' || p.scenario=='B') // Alternate per station on same BSS in these scenarios
	    {
	      NetDeviceContainer halfDevices;
	      for(uint16_t k = i; k<BSSdevices[i].GetN(); k+=2) { halfDevices.Add(BSSdevices[i].Get(k)); }
	      SetEICW(halfDevices);
	    }
	  else	// Each BSS operating with the same slot time in these scenarios
	    {
	      if(i>0) { SetEICW(BSSdevices[i]); } // Maintain Access Point in legacy mode
	    }
	}
    }

  //---------------------------------------------------------------------------------
  // Configure Internet access infrastructure on each access point
  //---------------------------------------------------------------------------------

  //Collect an adjacency list of AP nodes for the P2P topology
  std::vector<NodeContainer> nodeAdjacencyList (APSnodes.GetN ());
  for(uint32_t i=0; i<nodeAdjacencyList.size (); ++i)
    {
      nodeAdjacencyList[i] = NodeContainer (ISPnode, APSnodes.Get (i));
    }

  // Create channels for AP's Internet access links
  PointToPointHelper p2pISP;
  p2pISP.SetDeviceAttribute ("DataRate", StringValue ("100Mbps"));
  p2pISP.SetChannelAttribute ("Delay", StringValue ("1ms"));
  std::vector<NetDeviceContainer> deviceAdjacencyList (APSnodes.GetN());
  for(uint32_t i=0; i<deviceAdjacencyList.size (); ++i)
    {
      deviceAdjacencyList[i] = p2pISP.Install (nodeAdjacencyList[i]);
    }

  //---------------------------------------------------------------------------------
  // Setup Internet Stack parameters
  //---------------------------------------------------------------------------------
  if(verbose) NS_LOG_DEBUG ("Setup Internet Stack...");
  InternetStackHelper internet;
  Ipv4AddressHelper ipv4;
  Ipv4StaticRoutingHelper ipv4Routing;
  NodeContainer AllNodes(ISPnode, APSnodes, STAnodes);
  internet.Install(AllNodes);

  // Define IP address and routes on each BSS
  ipv4.SetBase("192.168.0.0", "255.255.255.0");
  for(uint8_t i = 0; i < p.numBSS; i++)
    {
      ipv4.Assign(BSSdevices[i]); // Set IP address

      // Define default gateway on each STA --> AP
      Ptr<Ipv4> ipv4AP = BSSnodes[i].Get(0)->GetObject<Ipv4>();
      Ipv4Address addrAP = ipv4AP->GetAddress(1,0).GetLocal();
      for(uint8_t j = 1; j < BSSnodes[i].GetN(); j++)
        {
          Ptr<Ipv4> ipv4sta = BSSnodes[i].Get(j)->GetObject<Ipv4>();
          Ptr<Ipv4StaticRouting> staticRoutingSta = ipv4Routing.GetStaticRouting(ipv4sta);
          staticRoutingSta->SetDefaultRoute(addrAP,1);
        }
      ipv4.NewNetwork();
    }

  // Define IP addresses and routes on ISP infrastructure
  ipv4.SetBase("10.0.0.0", "255.255.255.252");

  for(uint32_t i=0; i<APSnodes.GetN(); ++i)
    {
      // Set Address to ISP link
      ipv4.Assign (deviceAdjacencyList[i]);
      ipv4.NewNetwork();

      // Get ISP and Client (AP) Addresses and Interfaces
      Ptr<Ipv4> ipv4Gateway = nodeAdjacencyList[i].Get(0)->GetObject<Ipv4>();
      Ipv4Address addrGateway = ipv4Gateway->GetAddress(1+i,0).GetLocal(); // Each interface has 1 AP
      Ptr<Ipv4> ipv4Client = nodeAdjacencyList[i].Get(1)->GetObject<Ipv4>();
      uint32_t interfaceIndex = ipv4Client->GetNInterfaces()-1; // Last interface of AP
      Ipv4Address addrClient = ipv4Client->GetAddress(interfaceIndex,0).GetLocal();

      // Set default route on client to the last interface of AP
      Ptr<Ipv4StaticRouting> clientRoutes = ipv4Routing.GetStaticRouting(ipv4Client);
      clientRoutes->SetDefaultRoute(addrGateway,interfaceIndex);

      // Set route to client network
      std::ostringstream clientnet;
      clientnet<<"192.168."<<i<<".0";
      // TODO: replace this line with network address of WiFi device from AP

      Ptr<Ipv4StaticRouting> ispRoutes = ipv4Routing.GetStaticRouting(ipv4Gateway);
      ispRoutes->AddNetworkRouteTo(Ipv4Address(clientnet.str().c_str()),
				   Ipv4Mask("255.255.255.0"), addrClient ,(i+1));
    }

  //---------------------------------------------------------------------------------
  // Positioning nodes
  //---------------------------------------------------------------------------------
  if(verbose) NS_LOG_DEBUG ("Positioning nodes...");
  const double margin = 0;
  const double APheight = 3.0;
  const double STAheight = 1.0;
  const double BSSlenght = 10;

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> position = CreateObject<ListPositionAllocator>();

  position->Add(Vector(margin + p.numBSS * 2 * BSSlenght, margin, STAheight)); // ISP

  // positioning APs
  for(uint8_t i = 0; i < p.numBSS; i++)
    {
      double step = margin + BSSlenght + i * BSSlenght * 2;
      position->Add(Vector(step, step, APheight));
    }

  // Positioning STAs
  for(uint8_t i = 0; i < p.numBSS; i++)
    {
      double step = margin + BSSlenght + i * BSSlenght * 2;
      uint8_t thisBSS = p.numSTA / p.numBSS;
      if(i==0) thisBSS += (p.numSTA % p.numBSS);
      double angle = 2 * 3.141592653589 / thisBSS;
      for(uint8_t j = 1; j <= thisBSS; j++)
	{
	  double x = step + 10 * cos(j*angle);
	  double y = step - 10 * sin(j*angle);
	  position->Add(Vector(x, y, STAheight));
	}
    }

  mobility.SetPositionAllocator(position);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install(AllNodes);

  //----------------------------------------------------------------
  // Configure ECA if needed
  //----------------------------------------------------------------

  NodeContainer *wifiNodes = new NodeContainer();
  wifiNodes->Add(APSnodes);
  wifiNodes->Add(STAnodes);

  if (p.ECA)
  {
	  double setupTime = leadUp.GetSeconds() - 0.000001;
	  Simulator::Schedule (Seconds(setupTime), &SetupECA, wifiNodes);
  }

  //----------------------------------------------------------------
  // Configure TDuCSMA if needed
  //----------------------------------------------------------------

  if (p.TDuCSMA)
  {
	  double interval = 0.01;
	  Simulator::Schedule (leadUp, SetupTDuCSMA, STAnodes.GetN(), STAnodes, interval);
  }

  //----------------------------------------------------------------
  // Create packet sink applications
  //----------------------------------------------------------------
  if(verbose) NS_LOG_DEBUG ("Setup packet sink applications...");

  // Create a TCP packet sink on all nodes
  uint16_t tcpPort = 50000;
  Address sinkLocalAddress (InetSocketAddress (Ipv4Address::GetAny (), tcpPort));
  PacketSinkHelper sinkHelperTCP ("ns3::TcpSocketFactory", sinkLocalAddress);
  ApplicationContainer sinkAppTCP = sinkHelperTCP.Install (AllNodes);
  sinkAppTCP.Start (startSim);
  sinkAppTCP.Stop (endSim);

  // Create a UDP packet sink on all nodes
  uint16_t udpPort = 8000;
  PacketSinkHelper sinkHelperUDP ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny(), udpPort));
  ApplicationContainer sinkAppUDP = sinkHelperUDP.Install (AllNodes);
  sinkAppUDP.Start(startSim);
  sinkAppUDP.Stop(endSim);

  //----------------------------------------------------------------
  // Collect Data and Traces
  //----------------------------------------------------------------
  if(verbose) NS_LOG_DEBUG("Setup data collection and trace options");

  if(traceOn)
    {
      // Internet Stack trace
      AsciiTraceHelper ascii;
      Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream(version + "_trace.tr");
      internet.EnableAsciiIpv4All (stream);
    }

  //----------------------------------------------------------------
  // Configure Statistics Callback to this sample
  //----------------------------------------------------------------

  // List of nodes to connect on Statistics class and counters
  NodeContainer MonitoredNodes, NotMonitoredNodes;

  if(p.scenario == 'C' || p.scenario == 'D')
    {
      MonitoredNodes.Add(APSnodes);
      for(NodeContainer::Iterator node = STAnodes.Begin(); node != STAnodes.End(); node++)
	{
	  Ptr<NetDevice> nd = (*node)->GetDevice(0);
	  Ptr<WifiNetDevice> wifi = nd->GetObject<WifiNetDevice>();
	  Ptr<WifiMac> mac = wifi->GetMac();
	  Time slot = mac->GetSlot();

	  // Counter attached only to legacy nodes
	  if(p.scenario == 'C')
	    { if(slot == 9000 || slot == 20000) { MonitoredNodes.Add((*node)); } else { NotMonitoredNodes.Add((*node)); } }

	  // Counter attached only to new standard nodes
	  if(p.scenario == 'D')
	    { if(slot == 4500 || slot == 10000) { MonitoredNodes.Add((*node)); } else { NotMonitoredNodes.Add((*node)); } }
	}
      // Schedule data collect into measuring interval
      Simulator::Schedule (startSim - MilliSeconds(1), &ConnectCallbacks, MonitoredNodes, statistics);
      Simulator::Schedule (startSim - MilliSeconds(1), &ConnectPHYCallbacks, NotMonitoredNodes, statistics); // All nodes receive frames from monitored nodes at PHY layer
    }
  else
    {
      MonitoredNodes.Add(APSnodes);
      MonitoredNodes.Add(STAnodes);
      // Schedule data collect into measuring interval
      Simulator::Schedule (startSim - MilliSeconds(1), &ConnectCallbacks, MonitoredNodes, statistics);
    }

  std::ostringstream paramSink; // Callback Trace to Collect Received packets in PacketSink
  paramSink<<"/NodeList/"<<(ISPnode.Get (0)->GetId())<<"/ApplicationList/*/$ns3::PacketSink/Rx";
  Config::Connect (paramSink.str().c_str(), MakeCallback (&Statistics::AppRxCallback, statistics));

  // Calculate Throughput using Flowmonitor
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor;
  if(flowMonitor)  monitor = flowmon.InstallAll ();

  //----------------------------------------------------------------
  // Generate traffic to each sample increasing density stall by stall
  //----------------------------------------------------------------

  // Traffic parameters
  const bool uplink = true;

  // TODO: Check traffic profile suggest by TGax
  for(uint8_t i=0; i<STAnodes.GetN(); i++)
    {
      uint8_t EDCAac = p.ac;
      if(EDCAac==1) // if in mixed mode, each STA send a different type of traffic
	{
	  switch(i % 3)
	  {
	    case 0: EDCAac = AC_VO; break;
	    case 1: EDCAac = AC_VI; break;
	    case 2: EDCAac = AC_BE; break;
	  }
	}

      // If in a saturated channel option the generated traffic will be 100 mbits
      // If not in a saturated channel the generated traffic will be 25 mbits
      double kbpsPerStation = 36.0;
      kbpsPerStation = kbpsPerStation * 1024 / (p.numSTA/3);

      if(p.scenario=='E')	// Or if not saturated traffic, use typical
	{
	  switch(EDCAac)
	  {
	    case AC_VO: kbpsPerStation = 64; break;   // VOIP with G.711
	    case AC_VI: kbpsPerStation = 1024; break; // Interactive video conference (OnDemand Video between 1-4mbps in SD or 6-10mbps in HD)
	    case AC_BE: kbpsPerStation = kbpsPerStation - (p.numSTA/3 * 1024); break; // Total upload at 36 mbps
	  }
	}

      if(p.scenario=='A' || p.scenario=='B' || p.scenario=='E')
	{
	  // In scenario A or B or E - all nodes send data to ISP
	  GenerateTraffic(ISPnode, STAnodes.Get(i), startSim, p.sampleInterval, true, kbpsPerStation, uplink, p.packetSize, EDCAac, udpPort, 500000000);
	}
      else
	{
	  // In scenario C or D - half nodes send data to ISP and other half to invalid application
	  uint16_t port = 80;
	  for(NodeContainer::Iterator node = MonitoredNodes.Begin(); node != MonitoredNodes.End(); node++)
	    {
	      if((*node)->GetId() == STAnodes.Get(i)->GetId()) { port = udpPort; break; }
	    }
	  GenerateTraffic(ISPnode, STAnodes.Get(i), startSim, p.sampleInterval, true, kbpsPerStation, uplink, p.packetSize, EDCAac, port, 500000000);
	}
    }

  //----------------------------------------------------------------
  // Animation - To use with NetAnim
  //----------------------------------------------------------------
  if(verbose) NS_LOG_DEBUG("Define animation options");

  AnimationInterface anim (version + ".xml"); // Mandatory

  // Stations
  for (uint32_t i = 0; i < STAnodes.GetN (); i++)
    {
      anim.UpdateNodeSize (STAnodes.Get (i)->GetId(), 0.3, 0.3);
      std::ostringstream staID;
      staID<<(i+1);
      anim.UpdateNodeDescription (STAnodes.Get (i), staID.str().c_str());
      anim.UpdateNodeColor (STAnodes.Get (i)->GetId(), 255, 255, 0);  // Yellow
    }

  // Access Points
  for (uint8_t i = 0; i < p.numBSS; i++)
    {
      std::ostringstream apID;
      apID<<"AP"<<(i+1);
      anim.UpdateNodeDescription (APSnodes.Get (i), apID.str().c_str());
      anim.UpdateNodeColor (APSnodes.Get (i)->GetId(), 0, 255, 0);
      anim.UpdateNodeSize (APSnodes.Get (i)->GetId(), 0.35, 0.35);
    }

  // ISP
  anim.UpdateNodeDescription (ISPnode.Get (0), "ISP");
  anim.UpdateNodeColor (ISPnode.Get (0)->GetId(), 0, 0, 0);
  anim.UpdateNodeSize (ISPnode.Get (0)->GetId(), 0.5, 0.5);

  anim.EnablePacketMetadata (); // Optional
  //  anim.EnableIpv4RouteTracking ("routingtable-wireless.xml", Seconds (0), Seconds (1), Seconds (0.25)); //Optional
  //  anim.EnableWifiMacCounters (Seconds (0), Seconds (1)); //Optional
  //  anim.EnableWifiPhyCounters (Seconds (0), Seconds (1)); //Optional

  if(!animOn) { anim.SetStopTime(Seconds(0)); }

  //----------------------------------------------------------------
  // Processing
  //----------------------------------------------------------------

  if(verbose) NS_LOG_INFO("Running");
  Simulator::Stop(endSim);
  Simulator::Run ();

  //----------------------------------------------------------------
  // Statistics from flowMonitor
  //----------------------------------------------------------------
  if(flowMonitor)
    {
      Time delay = Seconds(0);
      Time jitter = Seconds(0);
      uint32_t rxPackets = 0; uint32_t lostPackets = 0;

      for(NodeContainer::Iterator sta = STAnodes.Begin(); sta != STAnodes.End(); sta++)
	{
	  // Get source and destination addresses
	  Ptr<Ipv4> ipv4source = (*sta)->GetObject<Ipv4>();
	  Ipv4Address addrSource = ipv4source->GetAddress(1,0).GetLocal(); // First interface of source node

	  Ptr<Ipv4> ipv4destination = ISPnode.Get(0)->GetObject<Ipv4>();
	  Ipv4Address addrDestination = ipv4destination->GetAddress(1,0).GetLocal(); // First interface of destination node

	  // Classify and filter the collected flowMonitor and show statistics
	  monitor->CheckForLostPackets ();
	  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
	  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
	  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
	    {
	      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
	      if ((t.sourceAddress == addrSource && t.destinationAddress == addrDestination))
		{
		  delay += i->second.delaySum;
		  jitter += i->second.jitterSum;
		  rxPackets += i->second.rxPackets;
		  lostPackets += i->second.lostPackets;
		}
	    }
	}
      NS_LOG_INFO ("Flow Monitor Lost Packets = " << lostPackets );
    }

  //----------------------------------------------------------------
  // Clean environment to prepare next turn and return statistics from this
  //----------------------------------------------------------------
  Simulator::Destroy ();
  return;
}

//------------------------------------------------------------------------
// End Experiment
//------------------------------------------------------------------------


int
main (int argc, char *argv[])
{
  //----------------------------------------------------------------
  // Define Parameters
  //----------------------------------------------------------------
  NS_LOG_INFO ("Initializing...");

  // Environment Setup
  Time::SetResolution (Time::NS);
  LogComponentEnable ("CI927RAS", LOG_LEVEL_INFO);

  // Script parameters
  parameters param;
  param.packetSize = 1000;
  param.sampleInterval = 10;
  param.repetitions = 35;
  param.RTSon = false;
  param.legacy = false;
  param.numBSS = 2;
  param.ECA = false;
  param.TDuCSMA = false;
  param.scenario = 'E';

  uint8_t minSTA = 3;
  uint8_t maxSTA = 70;

  bool trace = false;
  bool netAnim = false;
  bool flowMonitor = false;
  bool generatePDF = true;
  bool verbose = false;

  CommandLine cmd;
  cmd.AddValue ("minSTA", "Max number of stations", minSTA);
  cmd.AddValue ("maxSTA", "Max number of stations", maxSTA);
  cmd.AddValue ("numBSS", "Number of BSS where stations are associated", param.numBSS);
  cmd.AddValue ("packetSize", "Packet size in generated traffic", param.packetSize);
  cmd.AddValue ("sampleInterval", "Time on each measuring interval (seconds)", param.sampleInterval);
  cmd.AddValue ("repeat", "How many times must run each sample", param.repetitions);
  cmd.AddValue ("rtsOn", "Use RTS/CTS protection or basic mode", param.RTSon);
  cmd.AddValue ("legacy", "Saturated channel scenery or normal scenery", param.legacy);
  cmd.AddValue ("ECA", "CSMA/ECA protocol activated", param.ECA);
  cmd.AddValue ("TDuCSMA", "TDuCSMA protocol activated", param.TDuCSMA);
  cmd.AddValue ("scenario", "Use A for Saturated, B for non Saturated or C for mixed only", param.scenario);

  cmd.AddValue ("trace", "Active traces to get a register of traffic", trace);
  cmd.AddValue ("netAnim", "Generate XML animation files to NetAnim", netAnim);
  cmd.AddValue ("flowMonitor", "Show FlowMonitor report after each simulation", flowMonitor);
  cmd.AddValue ("generatePDF", "Generate Graph in PDF format (not EPS)", generatePDF);
  cmd.AddValue ("verbose", "Saturated channel scenery or normal scenery", verbose);
  cmd.Parse (argc,argv);

  // Check arguments from command line
  if(param.packetSize<1 || param.packetSize>8000) return EXIT_FAILURE;
  if(param.sampleInterval<1) return EXIT_FAILURE;
  if(param.repetitions<1 || param.repetitions>100) return EXIT_FAILURE;
  if(param.numBSS < 1 || param.numBSS>2) return EXIT_FAILURE;
  if(param.scenario < 'A' || param.scenario > 'E') return EXIT_FAILURE;

  if(minSTA < 1 || minSTA>254) return EXIT_FAILURE;
  if(maxSTA < 1 || maxSTA>254 || maxSTA < minSTA) return EXIT_FAILURE;

  //----------------------------------------------------------------
  // Run simulation for each situation
  //----------------------------------------------------------------

  // Scenario Saturated
  if(param.scenario == 'A')
  {
	  for(double slot = 0; slot<10; slot+=4.5)
	  {
		  param.slotTime = abs(slot);
		  std::vector<Statistics> statistics;
		  std::vector<std::string> legends;

		  for(param.ac=0; param.ac<4; param.ac++)
		  {
			  std::string legend = AccessCategory(param);
			  Statistics stats;
			  for(param.numSTA=minSTA; param.numSTA<=maxSTA; param.numSTA++)
			  {
				  const std::string filename = Filename(param);
				  NS_LOG_INFO("\nPreparing simulation " + filename);
				  if(!CheckFile(param, &stats)) // Look for CSV file with Statistics
				  {
					  // repeat n times each sample with different seed run
					  for(uint8_t j=0; j<param.repetitions; j++)
					  {
						  stats.NextSample();
						  MySim(&stats, param, j, trace, netAnim, flowMonitor, verbose);
					  }
					  stats.SaveStatistics(param.numSTA, param.sampleInterval, filename);
				  }
			  }
			  statistics.push_back(stats);
			  legends.push_back(legend);
		  }
		  param.ac=255;
		  param.numSTA = maxSTA;
		  const std::string filename = Filename(param);
		  plot(filename, statistics, legends, maxSTA, generatePDF);
	  }
  }

  // non saturated scenario
  if(param.scenario == 'B')
    {
      for(param.ac=0; param.ac<4; param.ac++)
        {
          std::vector<Statistics> statistics;
          std::vector<std::string> legends;

          for(double slot = 0; slot<10; slot+=4.5)
            {
              param.slotTime = abs(slot);
              std::string legend = SlotTime(param);
              Statistics stats;
              for(param.numSTA=minSTA; param.numSTA <= maxSTA; param.numSTA++)
        	{
        	  const std::string filename = Filename(param);
        	  NS_LOG_INFO("\nPreparing simulation " + filename);
        	  if(!CheckFile(param, &stats)) // Look for CSV file with Statistics
        	    {
        	      // repeat k times each sample with different seeds
        	      for(uint8_t k=0; k<param.repetitions; k++)
        		{
        		  stats.NextSample();
        		  MySim(&stats, param, k, trace, netAnim, flowMonitor, verbose);
        		}
        	      stats.SaveStatistics(param.numSTA, param.sampleInterval, filename);
        	    }
        	}
              statistics.push_back(stats);
              legends.push_back(legend);
            }
          param.slotTime = 255;
          param.numSTA = maxSTA;
	  const std::string filename = Filename(param);
          plot(filename, statistics, legends, maxSTA, generatePDF);
        }
    }

  // non saturated scenario
  if(param.scenario == 'C')
    {
      param.ac = AC_BE;
      param.slotTime = 0;

      std::vector<Statistics> statistics;
      std::vector<std::string> legends;

      for(uint8_t i=0; i<2; i++)
	{
          Statistics stats;
	  std::string legend;
	  if(i==0)
	    {
	      legend = param.legacy ? "20us" : "9us";
	    }
	  else
	    {
	      legend = param.legacy ? "10us" : "4.5us";
	      param.scenario = 'D';
	    }

	  if(minSTA%2 != 0) minSTA++; // In mixed scenario increment density in pairs of nodes (one on each slot time)

          for(param.numSTA=minSTA; param.numSTA <= maxSTA; param.numSTA+=2)
            {
              const std::string filename = Filename(param);
              NS_LOG_INFO("\nPreparing simulation " + filename);
              if(!CheckFile(param, &stats)) // Look for CSV file with Statistics
        	{
        	  // repeat k times each sample with different seeds
        	  for(uint8_t k=0; k<param.repetitions; k++)
        	    {
        	      stats.NextSample();
        	      MySim(&stats, param, k, trace, netAnim, flowMonitor, verbose);
        	    }
        	  stats.SaveStatistics(param.numSTA, param.sampleInterval, filename);
        	}
            }
          statistics.push_back(stats);
          legends.push_back(legend);
	}

      // Put a total with both types of nodes on same counters
      param.scenario = 'A';
      Statistics stats;
      std::string legend = "total";
      for(param.numSTA=minSTA; param.numSTA<=maxSTA; param.numSTA+=2)
	{
	  const std::string filename = Filename(param);
	  NS_LOG_INFO("\nPreparing simulation " + filename);
	  if(!CheckFile(param, &stats)) // Look for CSV file with Statistics
	    {
	      // repeat n times each sample with different seed run
	      for(uint8_t j=0; j<param.repetitions; j++)
		{
		  stats.NextSample();
		  MySim(&stats, param, j, trace, netAnim, flowMonitor, verbose);
		}
	      stats.SaveStatistics(param.numSTA, param.sampleInterval, filename);
	    }
	}
      statistics.push_back(stats);
      legends.push_back(legend);

      // Plot the graph with statistics separated by slot time
      param.slotTime = 255;
      param.numSTA = maxSTA;
      param.scenario = 'C';
      const std::string filename = Filename(param);
      plot(filename, statistics, legends, maxSTA, generatePDF);
    }

  // Scenario EICW & ECA
  if(param.scenario == 'E')
  {
	  std::vector<Statistics> statistics;
	  std::vector<std::string> legends;

	  for(uint8_t i=0; i<7; i++)
	  {
		  std::string legend;
		  Statistics stats;

		  param.ac = 1;

		  switch(i)
		  {
		  case 0:	param.slotTime = 9; param.ECA = false; param.TDuCSMA = false; legend = "NORMAL"; break;
		  case 1:	param.slotTime = 4; param.ECA = false; param.TDuCSMA = false; legend = "EICW"; break;
		  case 2:	param.slotTime = 9; param.ECA = true; param.TDuCSMA = false; legend = "CSMA/ECA"; break;
		  case 3:	param.slotTime = 9; param.ECA = false; param.TDuCSMA = true; legend = "TDuCSMA"; break;
		  case 4:	param.slotTime = 4; param.ECA = true; param.TDuCSMA = false; legend = "EICW+ECA"; break;
		  //case 5:	param.slotTime = 9; param.ECA = true; param.TDuCSMA = true; legend = "ECA+TDu"; break;
		  case 6:	param.slotTime = 4; param.ECA = false; param.TDuCSMA = true; legend = "EICW+TDu"; break;
		  case 5:	param.slotTime = 4; param.ECA = true; param.TDuCSMA = true; legend = "EICW+ECA+TDu"; break;
		  }

		  for(param.numSTA=minSTA; param.numSTA<=maxSTA; param.numSTA+=3)
		  {
			  const std::string filename = Filename(param);
			  NS_LOG_INFO("\nPreparing simulation " + filename);
			  if(!CheckFile(param, &stats)) // Look for CSV file with Statistics
			  {
				  // repeat n times each sample with different seed run
				  for(uint8_t j=0; j<param.repetitions; j++)
				  {
					  stats.NextSample();
					  MySim(&stats, param, j, trace, netAnim, flowMonitor, verbose);
				  }
				  stats.SaveStatistics(param.numSTA, param.sampleInterval, filename);
			  }
		  }
		  statistics.push_back(stats);
		  legends.push_back(legend);

	  }

	  param.ac=255;
	  param.numSTA = maxSTA;
	  const std::string filename = Filename(param);
	  plot(filename, statistics, legends, maxSTA, generatePDF);

  }



  NS_LOG_INFO ("\nFinished.");
}
