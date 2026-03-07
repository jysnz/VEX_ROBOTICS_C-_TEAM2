#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

// ============== AUTONOMOUS ROUTINES ==============

/**
 * Two vs Two (2v2) autonomous routine
 * Main scoring strategy for match play
 */
void twoVtwo();

/**
 * Two vs Two with matchloader pickup
 * Extended version that gathers additional game pieces
 */
void twovtwoWithMatchload();

/**
 * Skills challenge routine v3
 * Optimized for maximum point scoring in skills
 */
void skillsV3();

/**
 * Skills challenge routine v2
 * Previous version of skills routine
 */
void skillsV2();

/**
 * Skills challenge routine
 * Original skills implementation
 */
void skills();

/**
 * Debug/test autonomous routine
 * Used for testing specific movements and mechanisms
 */
void debug();

/**
 * Park routine
 * Minimal movement to park safely for scoring points
 */
void park();

/**
 * Simple motor test
 * Runs left motors for diagnostic purposes
 */
void test();

#endif // AUTONOMOUS_H
