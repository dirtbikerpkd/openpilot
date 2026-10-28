![](https://user-images.githubusercontent.com/47793918/233812617-beab2e71-57b9-479e-8bff-c3931347ca40.png)

## 🔧 About this fork (dirtbikerpkd / `rdx-torque-tuning`)
This is a personal fork of [sunnypilot](https://github.com/sunnyhaibin/sunnypilot) (itself a fork of comma.ai's openpilot), tuned for an **Acura RDX 3G** (`ACURA_RDX_3G`) on a **comma four**. It is not an official sunnypilot or comma release; use it at your own risk and see the license notes below.

**What's different from upstream sunnypilot**
* `opendbc_repo` points at [dirtbikerpkd/opendbc-rdx-tuning](https://github.com/dirtbikerpkd/opendbc-rdx-tuning) (branch `rdx-torque-tuning`) instead of the stock opendbc.
* RDX 3G steering tune: doubled `STEER_DELTA_UP/DOWN` ramp rate, `STEER_THRESHOLD` raised to 1500, and `steerActuatorDelay` of 0.15s.
* RDX 3G `LOW_SPEED_LOCKOUT` reported by the EPS above minimum steer speed is raised as `steerFaultTemporary`, so a silent loss of EPS assist alerts instead of being ignored.
* Honda-specific CAN validity logging, plus extra Honda fingerprints (2026 Civic Hatchback ICE, 2027 HR-V).
* Boot path synced from `rdx-tuning-202609-sp`: AGNOS 19.7 manifest and launcher fixes.
* Bosch-A radar decoder (reads the stock radar's object list, RX only; on by default via the `HondaBoschARadar` param) with a `radard` lateral-match gate. It only decodes while openpilot longitudinal is off, because enabling openpilot long silences the radar ECU.
* Lead vehicle distance / speed / time-to-collision shown next to the lead chevron on the comma four screen, even without openpilot longitudinal control.
* Custom P@ logo for the loading spinner and the bouncing screensaver. The power-on comma logo comes from AGNOS firmware and is not changed by this repo.

**Install (comma four)**
1. On the device, choose *Custom Software* at setup, or Settings → Software → *Uninstall* and reinstall.
2. When prompted for the software URL, enter just the user/branch:
   ```
   dirtbikerpkd/rdx-torque-tuning
   ```
3. The device downloads the branch and reboots. This branch ships **without a prebuilt**, so the first boot compiles on the device (scons) and shows a progress spinner. It can take a long time; don't power-cycle it.
4. If the device gets stuck on the comma logo, SSH in and check `cat /VERSION` and `tail -50 /tmp/launch_log`. The branch expects AGNOS **19.7**.

**Branches**
* `rdx-torque-tuning` is the default branch and the one to install.
* `rdx-tuning-202609-sp` is a separate line based on a newer sunnypilot snapshot. It also carries the Honda steering torque learner and Nidec work, which are not in `rdx-torque-tuning`.

**Notes**
* The RDX 3G runs a PID lateral tune. `EnforceTorqueControl` and `NeuralNetworkLateralControl` are sunnypilot settings you can try, but they are not enabled by this fork.

---

## 🌞 What is sunnypilot?
[sunnypilot](https://github.com/sunnyhaibin/sunnypilot) is a fork of comma.ai's openpilot, an open source driver assistance system. sunnypilot offers the user a unique driving experience for over 300+ supported car makes and models with modified behaviors of driving assist engagements. sunnypilot complies with comma.ai's safety rules as accurately as possible.

## 💭 Join our Community Forum
Join the official sunnypilot community forum to stay up to date with all the latest features and be a part of shaping the future of sunnypilot!
* https://community.sunnypilot.ai/

## Documentation
https://docs.sunnypilot.ai/ is your one stop shop for everything from features to installation to FAQ about the sunnypilot

## 🚘 Running on a dedicated device in a car
First, check out this list of items you'll need to [get started](https://community.sunnypilot.ai/t/getting-started-using-sunnypilot-in-your-supported-car/251).

## Installation
Next, refer to the sunnypilot community forum for [installation instructions](https://community.sunnypilot.ai/t/read-before-installing-sunnypilot/254), as well as a complete list of [Recommended Branch Installations](https://community.sunnypilot.ai/t/recommended-branch-installations/235).

## 🎆 Pull Requests
We welcome both pull requests and issues on GitHub. Bug fixes are encouraged.

Pull requests should be against the most current `master` branch.

## 📊 User Data

By default, sunnypilot uploads the driving data to comma servers. You can also access your data through [comma connect](https://connect.comma.ai/).

sunnypilot is open source software. The user is free to disable data collection if they wish to do so.

sunnypilot logs the road-facing camera, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The driver-facing camera and microphone are only logged if you explicitly opt-in in settings.

By using this software, you understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of comma. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to comma for the use of this data.

## Licensing

sunnypilot is released under the [MIT License](LICENSE). This repository includes original work as well as significant portions of code derived from [openpilot by comma.ai](https://github.com/commaai/openpilot), which is also released under the MIT license with additional disclaimers.

The original openpilot license notice, including comma.ai’s indemnification and alpha software disclaimer, is reproduced below as required:

> openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.
>
> Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.
>
> **THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
> YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
> NO WARRANTY EXPRESSED OR IMPLIED.**

For full license terms, please see the [`LICENSE`](LICENSE) file.

## 💰 Support sunnypilot
If you find any of the features useful, consider becoming a [sponsor on GitHub](https://github.com/sponsors/sunnyhaibin) to support future feature development and improvements.


By becoming a sponsor, you will gain access to exclusive content, early access to new features, and the opportunity to directly influence the project's development.


<h3>GitHub Sponsor</h3>

<a href="https://github.com/sponsors/sunnyhaibin">
  <img src="https://user-images.githubusercontent.com/47793918/244135584-9800acbd-69fd-4b2b-bec9-e5fa2d85c817.png" alt="Become a Sponsor" width="300" style="max-width: 100%; height: auto;">
</a>
<br>

<h3>PayPal</h3>

<a href="https://paypal.me/sunnyhaibin0850" target="_blank">
<img src="https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif" alt="PayPal this" title="PayPal - The safer, easier way to pay online!" border="0" />
</a>
<br></br>

Your continuous love and support are greatly appreciated! Enjoy 🥰

<span>-</span> Jason, Founder of sunnypilot
