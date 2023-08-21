#include "pdb.h"

#include "../utils/logging.h"

#include <queue>
#include <limits> 

using namespace std;

namespace planopt_heuristics {

/*
  An entry in the queue is a tuple (h, i) where h is the goal distance of state i.
  See comments below for details.
*/
using QueueEntry = pair<int, int>;

PatternDatabase::PatternDatabase(const TNFTask &task, const Pattern &pattern)
    : projection(task, pattern) {
    /*
      We want to compute goal distances for all abstract states in the
      projected task. To do so, we start by assuming every abstract state has
      an infinite goal distance and then do a backwards uniform cost search
      updating the goal distances of all encountered states.

      Instead of searching on the actual states, we use perfect hashing to
      run the search on the hash indices of states. To go from a state s to its
      index use rank(s) and to go from an index i to its state use unrank(i).
    */
    const TNFTask &projected_task = projection.get_projected_task();
    distances.resize(projected_task.get_num_states(), numeric_limits<int>::max());

    /*
      Priority queues usually order entries so the largest entry is the first.
      By using the comparator greater<T> instead of the default less<T>, we
      change the ordering to sort the smallest element first.
    */ 
    priority_queue<QueueEntry, vector<QueueEntry>, greater<QueueEntry>> queue;
    /*
      Note that we start with the goal state to turn the search into a regression.
      We also have to switch the role of precondition and effect in operators
      later on. This is sufficient to turn the search into a regression since
      the task is in TNF.
    */
    queue.push({0, projection.rank_state(projected_task.goal_state)});

    // TODO: add your code for exercise (b) here.
    while(!queue.empty()){
      QueueEntry front = queue.top();
      queue.pop();

      if(front.first >= distances[front.second]){
        continue;
      }

      distances[front.second] = front.first;

      TNFState state = projection.unrank_state(front.second);
      for(const TNFOperator &oper : projected_task.operators){
        //vector<int> succ_state = projection.unrank_state(front.second);
        bool bAplicable = true;
        for(const TNFOperatorEntry &entry : oper.entries){
          if(state[entry.variable_id] != entry.effect_value){
            bAplicable = false;
            break;
          }
          //succ_state[entry.variable_id] = entry.precondition_value;
        }

        if(bAplicable){
          TNFState succ_state = projection.unrank_state(front.second);

          for(const TNFOperatorEntry &entry : oper.entries){
            succ_state[entry.variable_id] = entry.precondition_value;
          }

          QueueEntry successor;
          successor.first = front.first + oper.cost;
          successor.second = projection.rank_state(succ_state);
          
          // if better cost found then add to queue and update value
          if(distances[successor.second] > successor.first){
            queue.push(successor);
          }
        }

      }
    }
}

int PatternDatabase::lookup_distance(const TNFState &original_state) const {
    TNFState abstract_state = projection.project_state(original_state);
    int index = projection.rank_state(abstract_state);
    return distances[index];

}

}
