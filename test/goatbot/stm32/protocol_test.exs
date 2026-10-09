defmodule Goatbot.STM32.ProtocolTest do
  use ExUnit.Case, async: true

  alias Goatbot.STM32.Protocol

  # The expected wire strings mirror the firmware's own test suite in
  # nsk_stm32/protocol/src/lib.rs so this stays a faithful client.

  describe "encode/1" do
    test "encodes commands to their bare wire line (no terminator)" do
      assert IO.iodata_to_binary(Protocol.encode({:set_rtc, 1_757_260_800})) == "RTC 1757260800"
      assert IO.iodata_to_binary(Protocol.encode(:get_rtc)) == "RTC?"
      assert IO.iodata_to_binary(Protocol.encode({:sleep, 300})) == "SLEEP 300"
      assert IO.iodata_to_binary(Protocol.encode({:sleep, 0})) == "SLEEP 0"
      assert IO.iodata_to_binary(Protocol.encode(:get_wake_reason)) == "WAKE?"
      assert IO.iodata_to_binary(Protocol.encode(:get_power)) == "PWR?"
      assert IO.iodata_to_binary(Protocol.encode({:set_charge_thresholds, 3500, 3900})) ==
               "CHGLIM 3500 3900"

      assert IO.iodata_to_binary(Protocol.encode({:set_charge_thresholds, -100, -50})) ==
               "CHGLIM -100 -50"

      assert IO.iodata_to_binary(Protocol.encode({:set_charging, true})) == "CHG 1"
      assert IO.iodata_to_binary(Protocol.encode({:set_charging, false})) == "CHG 0"
      assert IO.iodata_to_binary(Protocol.encode(:enter_bootloader)) == "BOOTLOADER"
    end
  end

  describe "decode/1 acknowledgements" do
    test "decodes every simple acknowledgement" do
      assert Protocol.decode("OK RTC") == {:ok, :ack_set_rtc}
      assert Protocol.decode("OK SLEEP") == {:ok, :ack_sleep}
      assert Protocol.decode("OK CHGLIM") == {:ok, :ack_set_charge_thresholds}
      assert Protocol.decode("OK CHG") == {:ok, :ack_set_charging}
      assert Protocol.decode("OK BOOTLOADER") == {:ok, :ack_enter_bootloader}
    end

    test "decodes the RTC query reply, distinguishing it from the set ack" do
      assert Protocol.decode("OK RTC 1757260800") == {:ok, {:rtc, 1_757_260_800}}
      assert Protocol.decode("OK RTC 0") == {:ok, {:rtc, 0}}
    end

    test "decodes every wake reason" do
      assert Protocol.decode("OK WAKE pwr") == {:ok, {:wake_reason, :power_on}}
      assert Protocol.decode("OK WAKE btn") == {:ok, {:wake_reason, :button}}
      assert Protocol.decode("OK WAKE rtc") == {:ok, {:wake_reason, :rtc}}
      assert Protocol.decode("OK WAKE unk") == {:ok, {:wake_reason, :unknown}}
    end

    test "decodes power metrics in declaration order, i_batt signed" do
      assert Protocol.decode("OK PWR 3812 -142 1 1") ==
               {:ok,
                {:power,
                 %{v_batt_mv: 3812, i_batt_ma: -142, usb_present: true, chrg_en: true}}}

      assert Protocol.decode("OK PWR 0 0 0 0") ==
               {:ok,
                {:power, %{v_batt_mv: 0, i_batt_ma: 0, usb_present: false, chrg_en: false}}}
    end
  end

  describe "decode/1 events" do
    test "decodes button events" do
      assert Protocol.decode("EVT BTN 0 1") == {:event, {:button, 0, true}}
      assert Protocol.decode("EVT BTN 2 0") == {:event, {:button, 2, false}}
    end
  end

  describe "decode/1 errors" do
    test "decodes error lines with verb and reason" do
      assert Protocol.decode("ERR BOGUS badcmd") == {:error, {"BOGUS", :badcmd}}
      assert Protocol.decode("ERR CHGLIM range") == {:error, {"CHGLIM", :range}}
      assert Protocol.decode("ERR - toolong") == {:error, {"-", :toolong}}
      assert Protocol.decode("ERR PWR? badargs") == {:error, {"PWR?", :badargs}}
    end
  end

  describe "decode/1 unrecognized input" do
    test "returns :unknown for lines that do not match the grammar" do
      assert Protocol.decode("garbage line") == {:unknown, "garbage line"}
      assert Protocol.decode("") == {:unknown, ""}
      assert Protocol.decode("OK") == {:unknown, "OK"}
    end
  end
end
