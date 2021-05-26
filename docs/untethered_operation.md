
# Untethered Operation

As a mobile manipulator, the Stretch RE1 can only go so far when tethered to the monitor, keyboard, and mouse setup. This guide will explain three methods of setting up the Stretch RE1 for untethered usage.

These methods typically require a wireless network, but it is possible to set up any of these methods without a wireless network by [setting up a hotspot](#hotspot).

## Remote Desktop

#### Requirements

This is the recommended approach if you are running Windows or MacOS. This method requires a Virtual Network Computing (VNC) package. Using any of the free or paid options available for Windows, MacOS, and Chrome will be fine since they all use the Remote Frame Buffer (RFB) protocol to communicate with the robot. If you're using Ubuntu, Remmina Remote Desktop Client will be installed by default.

#### How To

While the Stretch RE1 is tethered to the monitor, keyboard, and mouse setup, first verify that the robot is connected to the wireless network then install Vino VNC server using the following command:

```bash
sudo apt install vino
```

Go to System Settings. Select the Sharing tab and turn it on then, turn on Screen Sharing and choose a password. If you plan to connect to the robot from a Windows or MacOS machine, then open a terminal and run the following command.

```bash
sudo gsettings set org.gnome.Vino require-encryption false
```

Finally, we need the robot's IP address, username, and password. Open a terminal and run `ifconfig`, which will print out the network information of the machine. In the wireless section (typically named wlp2s0), look for something that looks like "inet 10.0.0.15". The four numbers represent the IP address of the robot on the local network. The robot's default username and password is printed on papers that came in the tools box alongside the robot.

VNC will only function properly with an external display attached to the robot. Using a dummy HDMI dongle when operating the robot untethered via VNC is recommended. One possible dummy HDMI dongle can be found on Amazon [here](https://www.amazon.com/gp/product/B07BBT9NCZ/). On your computer, connect to the same wireless network as the robot and open the VNC package being used. Using the robot's IP address and username, initialize a new connection to the robot. The robot's desktop will open in a new window.

## SSH & X Server

#### Requirements

This is the recommended approach if you are running an Unix-based operating system, like Ubuntu or Arch Linux. This method requires both SSH and X Server to be installed. While most Unix-based operating systems have both installed by default, MacOS will only have SSH installed and Windows has neither installed by default. It is possible to install these tools for MacOS or Windows.

#### How To

While the Remote Desktop approach is easy to set up, graphics and interaction with the remote desktop is often slow. In this method, we will use SSH and X Server to accomplish the same a bit faster. SSH stands for Secure Shell, enabling one to remotely use the terminal (shell) of another machine. X Server is used on many Unix variants to render the Windowed GUI of applications. With SSH and X Server, it is possible to render the Windowed GUI of an application running on a remote machine on your machine's screen.

The first step is to identify the robot's IP address on the local network. While the Stretch RE1 is tethered to the monitor, keyboard, and mouse, verify that the robot is connected to a wireless network. Then, open a terminal and run `ifconfig`, which will print out the network information of the machine. In the wireless section (typically named wlp2s0), look for something that looks like "inet 10.0.0.15". The four numbers represent the IP address of the robot on the local network. Using any other machine on the same local network, I can SSH into the robot using this IP address. Take note of the username and password of the robot. The default combo is printed on papers that came in the tools box alongside the robot.

To SSH into the robot, run the following. It will require the password and may ask you to add the robot to the known hosts.

```bash
ssh -X username@ip-address
```

Now that you're SSH-ed into the robot, you can disconnect any wires from the robot. You can accomplish any of the same tasks through the terminal. For example, you can type in `ipython` and try out all of the same python commands from stretch_body that were explained in the [Getting Started Guide](quick_start_guide.md#start-coding).

Furthermore, Windowed GUI applications that would have displayed on the monitor will now display on your SSH-ed machine. For example, we can open Rviz to visualize what the robot is seeing. Open two terminals and SSH into the robot as explained above. In the first, run `roslaunch stretch_core stretch_driver.launch`. You should see some information print out in the terminal. In the second, run `rviz`. A window will pop up and information about the robot can be visualized by clicking on `Add -> RobotModel` and `Add -> By Topic -> /Scan`. Additional information on how to use ROS tools can be found in [ROS's tutorials](http://wiki.ros.org/ROS/Tutorials) or in our [guides on using stretch_ros](README.md#ros-interface).

#### Moving files to/from the robot wirelessly

It's common to need to move files to/from the robot wirelessly and a tool similar to SSH can help with this: Secure Copy (SCP). This tool will not copy directories, so you may need to zip your files up prior to copying them over.

To send the files from your computer to the robot, run:
```bash
scp ./filename username@ip-address:~/path/to/put/it/
```

To copy the files from the robot to your computer, run the reverse:
```bash
scp username@ip-address:/path/to/filename ~/path/to/put/it/
```

## ROS Remote Master

#### Requirements

This is the recommended approach if you are running Ubuntu 16.04/18.04 with ROS kinetic/melodic installed. This method will utilize the local installation of ROS tools, such as Rviz, rostopic, and rosservice, while retrieving data from the robot.

#### How To

If you are developing ROS code to test on the Stretch RE1 and you already have ROS installed on your local Ubuntu machine, then there is an easier way of using Rviz than the method described in [SSH & X Server](#ssh-x-server). In the ROS world, this concept is known as "remote master".

First, start the ROS launch files on the robot as you normally would. On you local machine, run:

```bash
export ROS_MASTER_URI=http://username@ip-address:11311
```

Then you may use tools like Rviz, rostopic, or rosservice on your local machine as you would have on the robot. Additional information can be found in the ROS [Multiple Machines Tutorial](http://wiki.ros.org/ROS/Tutorials/MultipleMachines).

## Additional Ideas

Although the methods described above will enable you to wirelessly control the robot, there are a number of way to improve usability and security of your wireless connection. These ideas are listed here.

### Hotspot

Often the trouble with wirelessly controlling the robot is the network. If your network is using industrial security like 2-factor authentication, there may be trouble connecting the robot to the network. If the network is servicing a large number of users, the connection may feel sluggish. The alternative is to skip the network by connecting directly to the robot. After starting a hotspot on the robot, you can follow instructions for any of the methods described above to control the robot. The trade-off is that while connected to the robot's hotspot, you will be unable to connect to the internet.

To set up the robot's hotspot, visit the Ubuntu Wifi Settings page in the robot. Click on the hamburger menu in the top right and select "Enable hotspot". From your local machine, connect to the robot's hotspot and save the credentials. To change the hotspot's password or enable the hotspot automatically whenever the robot boots, see the following [Stackoverflow post](https://askubuntu.com/questions/500370/setting-up-wireless-hotspot-to-be-on-at-boot).

### VS Code Remote Development

It is possible to simultaneously develop code on the robot while running wireless experiments using the Remote Development Extension provided by the VS Code IDE. If you're already using the [VS Code IDE](https://code.visualstudio.com/), navigate to the Extensions tab and search for [Remote Development Extension by Microsoft](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack). After installing, click on a green button in the bottom left of the screen and then select "Remote-SSH: Connect to Host". Setting this up for the first time will require you to know the robot's IP address and username. Add a new host with the information. While connecting, VS Code will ask you for the password of the robot. Once you are connected, you can open any folder and edit the code remotely. Combined with the method explained in [SSH & X Server](#ssh-x-server), this is a powerful method of iteratively developing code while testing it.

### Static IP Address

Routers that serve wireless networks often dynamically assign IP address to machines that connect to the network. This means that your robot's IP address may have changed since the last time you turned it on. Since it becomes a pain to connect to the monitor, keyboard, and mouse setup every time to run `ifconfig`, many users prefer to assign the robot a static IP address. If you control the router, visit the router's settings page to set up the robot's static IP address. It is common at universities and companies to have staff dedicated to the management of the network. This staff will often be able to set up a static IP address for the robot.

### Public Key Authentication

The method of SSH described in [SSH & X Server](#ssh-x-server) is the most basic method of SSH-ing. There is a better and more-secure method of SSH-ing into the robot called Public Key Authentication. This method will allow multiple developers to SSH into the robot without having to share the robot's admin password.

The first step is to generate public and private keys on your computer. Linux and MacOS machines can simply open the terminal and run:

```bash
ssh-keygen
```

It will prompt you to enter a password. If you do, you'll need it to use the private key when you SSH into the robot. Next, we give the robot the public key. Linux and MacOS machines can run:

```bash
ssh-copy-id username@ip-address
```

This requires you to know the username and ip-address of the robot. Instructions on how to find this information is found in the [SSH & X Server](#ssh-x-server) section. You may now SSH into the robot as normal.
