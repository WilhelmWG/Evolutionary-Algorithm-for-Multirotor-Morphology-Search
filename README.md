<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a name="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->
<!-- ABOUT THE PROJECT -->
## About The Project
This project is an implementation of general multirotor dynamics and an evolutionary algorithm to search for and generate novel Micro Aerial Vehicles (MAVs). The designs are evolved to perform for a set of trajectories designed to mimic traversing an underground environment similar to what is expected in the DARPA subterranean challenge. Their fitness is evaluated based on their ability to follow these trajectories while maintaining designated full orientation. This involves minimizing the errors between the true trajectory and the desired one while simultaneously minimizing the amount of battery power expended.

## File contents
- GA.py contains the majority of parameters and function required to make a pygad.GA instance.
- MultiRotorDynamics.py defines the MultiRotor class, its dynamics and functions for simulating it following a trajectory.
- MotorRotorAnalysis.py defines dictionaries that contain all necessary information on motor-rotor combinations and batteries used in this project.
- Trajs.py defines all trajectories the MultiRotor tracks when evaluating its fitness.
- main.py runs a genetic algorithm, writes the best solution and GA_instance to files and plots the fitness, genes and 3D model of the MultiRotor.
- old_main.py runs a predefined MultiRotor for a single trajectory, primarily used for testing
plotting.py contains functions for plotting 2D and 3D representations of trajectories and a function for making a 3D model of a MultiRotor object.
- utils.py contains some helpful utility functions
- tests.py defines test for utility functions
- load_and_run.py loads a MultiRotor from a .txt file and simulates it tracking one trajectory and plots its performance and 3D model, used for evaluating results from a genetic algorithm run.

<!-- GETTING STARTED -->
## Getting Started

### Prerequisites

To run the files in this repository a python environment for version 3.11.5 can be created using the EvolveEnv.yml file using the following command

```sh
conda env create -f environment.yml
```

### Usage
One example of using this repository is as follows. 
1. Modify the parameters of GA.py to what you want to test for. 
2. Modify Trajs.py if new or modified trajectories are desired.
3. Run main.py
4. Modify load_and_run.py to point to the file path created by main.py.
5. Run load_and_run.py to see the performance of the best solution of the genetic algorithm.




<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/github_username/repo_name.svg?style=for-the-badge
[contributors-url]: https://github.com/github_username/repo_name/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/github_username/repo_name.svg?style=for-the-badge
[forks-url]: https://github.com/github_username/repo_name/network/members
[stars-shield]: https://img.shields.io/github/stars/github_username/repo_name.svg?style=for-the-badge
[stars-url]: https://github.com/github_username/repo_name/stargazers
[issues-shield]: https://img.shields.io/github/issues/github_username/repo_name.svg?style=for-the-badge
[issues-url]: https://github.com/github_username/repo_name/issues
[license-shield]: https://img.shields.io/github/license/github_username/repo_name.svg?style=for-the-badge
[license-url]: https://github.com/github_username/repo_name/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/linkedin_username
[product-screenshot]: images/screenshot.png
[Next.js]: https://img.shields.io/badge/next.js-000000?style=for-the-badge&logo=nextdotjs&logoColor=white
[Next-url]: https://nextjs.org/
[React.js]: https://img.shields.io/badge/React-20232A?style=for-the-badge&logo=react&logoColor=61DAFB
[React-url]: https://reactjs.org/
[Vue.js]: https://img.shields.io/badge/Vue.js-35495E?style=for-the-badge&logo=vuedotjs&logoColor=4FC08D
[Vue-url]: https://vuejs.org/
[Angular.io]: https://img.shields.io/badge/Angular-DD0031?style=for-the-badge&logo=angular&logoColor=white
[Angular-url]: https://angular.io/
[Svelte.dev]: https://img.shields.io/badge/Svelte-4A4A55?style=for-the-badge&logo=svelte&logoColor=FF3E00
[Svelte-url]: https://svelte.dev/
[Laravel.com]: https://img.shields.io/badge/Laravel-FF2D20?style=for-the-badge&logo=laravel&logoColor=white
[Laravel-url]: https://laravel.com
[Bootstrap.com]: https://img.shields.io/badge/Bootstrap-563D7C?style=for-the-badge&logo=bootstrap&logoColor=white
[Bootstrap-url]: https://getbootstrap.com
[JQuery.com]: https://img.shields.io/badge/jQuery-0769AD?style=for-the-badge&logo=jquery&logoColor=white
[JQuery-url]: https://jquery.com 
