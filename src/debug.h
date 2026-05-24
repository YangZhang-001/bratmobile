#ifndef DEBUG_H
#define DEBUG_H

#include "disturbance.h"
#include "graphTools.h"
#include <bits/stdc++.h>
#include <dirent.h>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

/**
 * @brief Class used to load data from the configurator
 * 
 */
class Logger
{
  protected:
    char fileName[60];
    FILE *f = NULL;
    int fileCount = 0; //files with the same name

  public:
    Logger () {}

    /**
	 * @brief Construct a new Logger object
	 * 
	 * @param new_folder folder where files will be dumped (no / at the end)
	 * @param _dir directory containing new_folder
	 * @param customName file prefix (/ must be at the beginning)
	 * @param dateOn whether to add today's date and time to file name
	 */
    Logger (const char *new_folder, const char *_dir = "/tmp",
            const char *customName = "/stats", bool dateOn = true)
    {
        init (new_folder, _dir, customName, dateOn);
    }

    ~Logger ()
    {
        if (NULL != f)
        {
            fclose (f);
        }
        f = NULL;
    }

    /**
	 * @brief 
	 * 
	 * @param format printf style e.g. "hello%s"
	 * @param ... other parameters
	 */
    bool log (const char *format, ...);

    const char *get_fileName () { return fileName; }

    /**
	 * @brief Returns a string with system architecture
	 */
    static const char *getSystemArchitecture ();

  protected:
    /**
	 * @brief Creates filename with today's date and time, name in format customdmy_hm.txt
	 * 
	 * @param custom custom
	 * @param name empty char array
	 */
    std::string file_dateTime (const char *custom, char name[80]);

    /**
	 * @see Logger
	 */
    void init (const char *new_folder, const char *_dir = NULL,
               const char *customName = "/stats", bool dateOn = false);
};

namespace debug
{

template <class T>
void print_graph (const T &g, const Disturbance &goal,
                  std::vector<vertexDescriptor> plan,
                  const vertexDescriptor &c)
{
    std::stringstream os;
    auto vs = boost::vertices (g);
    for (auto vi = vs.first; vi != vs.second; vi++)
    {
        auto es = boost::out_edges (*vi, g);
        if (*vi == c)
        {
            os << "!";
        }
        for (vertexDescriptor vp : plan)
        {
            if (*vi == vp)
            {
                os << "*";
            }
        }
        os << *vi << "-> ";
        for (auto ei = es.first; ei != es.second; ei++)
        {
            if (*ei != edgeDescriptor ())
            {
                os << (*ei).m_target << "(" << g[(*ei)].probability << ")";
            }
        }
        os << "\t(x=" << g[*vi].endPose.p.x << ", y= " << g[*vi].endPose.p.y
           << ", theta= " << g[*vi].endPose.q.GetAngle () << ")\n";
    }
    std::cout << os.str ();
}

template <class T>
void graph_file (const int &it, const T &g, const Disturbance &goal,
                 std::vector<vertexDescriptor> &plan,
                 const vertexDescriptor &c)
{
    char fileName[50];
    sprintf (fileName, "/tmp/graph%04i.txt", it);
    FILE *f = fopen (fileName, "w");
    auto vs = boost::vertices (g);
    for (auto vi = vs.first; vi != vs.second; vi++)
    {
        auto es = boost::out_edges (*vi, g);
        if (*vi == c)
        {
            fprintf (f, "!");
        }
        for (vertexDescriptor vp : plan)
        {
            if (*vi == vp)
            {
                fprintf (f, "*");
            }
        }
        fprintf (f, "%li -> ", (*vi));
        for (auto ei = es.first; ei != es.second; ei++)
        {

            fprintf (f, "%li (%f) ", (*ei).m_target, g[(*ei)].probability);
        }
        fprintf (f, "\t(x=%.3f, y= %.3f, theta= %.3f)\n", g[*vi].endPose.p.x,
                 g[*vi].endPose.p.y, g[*vi].endPose.q.GetAngle ());
    }
    fclose (f);
}
b2Vec2 GetWorldPoints (b2Body *, b2Vec2);

void print_pose (const b2Transform &p, const char *msg = NULL);

void print_matrix (const cv::Mat &);

std::vector<b2Vec2> GetBodies (b2World *);

void print_state_difference (const StateDifference &sd, vertexDescriptor v,
                             vertexDescriptor v1);

}

#endif
