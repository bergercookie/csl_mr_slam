Following work is not yet complete. Only working anisble-playbook is the ]push_code_to_robots.yml

Repository contains scripts and configuration files to deploy code, launch
various tasks (setup, execution of graphSLAM), and overall manage the set of
running graphSLAM agent (nodes)

Tasks that are handled are the following:

- Deployment of code
- Automatic compilation (after deployment)
- Setup of preliminary actions in the robots. Among others processes handling
    the following are launched:

    + Wheel/Motion control
    + Teleloperation
    + Sensor measurements acquisition (e.g. laser)
    + mr-graphSLAM node

Depending on the environment variables set in the corresponding host, the
appropriate roslaunch files are going to be used.
