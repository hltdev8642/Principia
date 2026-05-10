using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace principia {
namespace ksp_plugin_adapter {

class FlightPlanner : RequiredVesselSupervisedWindowRenderer {
  public FlightPlanner(PrincipiaPluginAdapter adapter,
                       PredictedVessel predicted_vessel) : base(
      adapter,
      predicted_vessel) {
    adapter_ = adapter;
    predicted_vessel_ = predicted_vessel;
    final_time_ = new DifferentialSlider(
        label               :
        L10N.CacheFormat("#Principia_FlightPlan_PlanLength"),
        unit                : null,
        log10_lower_rate    : log10_time_lower_rate,
        log10_upper_rate    : log10_time_upper_rate,
        min_value           : 10,
        max_value           : double.PositiveInfinity,
        formatter           : FormatPlanLength,
        parser              : TryParsePlanLength,
        field_width         : 7,
        display_zero_button : false);
    final_trajectory_analyser_ =
        new PlannedOrbitAnalyser(adapter, predicted_vessel);
  }

  public void RenderButton() {
    RenderButton(L10N.CacheFormat("#Principia_FlightPlan_ToggleButton"),
                 GUILayoutWidth(4));
  }

  public bool show_guidance => show_guidance_;

  public override void Load(ConfigNode node) {
    base.Load(node);

    string show_guidance_value = node.GetAtMostOneValue("show_guidance");
    if (show_guidance_value != null) {
      show_guidance_ = Convert.ToBoolean(show_guidance_value);
    }
  }

  public override void Save(ConfigNode node) {
    base.Save(node);

    node.SetValue("show_guidance", show_guidance_, createIfNotFound : true);
  }

  internal NavigationManoeuvre GetManœuvre(int i) {
    string vessel_guid = predicted_vessel?.id.ToString();
    if (vessel_guid == null) {
      Log.Fatal("there is no predicted vessel");
    }
    return plugin.FlightPlanGetManoeuvre(vessel_guid, i);
  }

  internal void ReplaceBurn(int i, Burn burn) {
    string vessel_guid = predicted_vessel?.id.ToString();
    if (vessel_guid == null) {
      return;
    }
    var status = plugin.FlightPlanReplace(vessel_guid, burn, i);
    UpdateStatus(status, i);
    ResetOptimizer(vessel_guid);
    if (burn_editors_?.Count > i && burn_editors_[i] is BurnEditor editor) {
      editor.Reset(plugin.FlightPlanGetManoeuvre(vessel_guid, i));
    }
  }

  internal void RequestEditorFocus(int i) {
    requested_editor_focus_index_ = i;
  }

  protected override string Title =>
      L10N.CacheFormat("#Principia_FlightPlan_Title");

  protected override void RenderWindowContents(int window_id) {
    // We must ensure that the GUI elements don't change between Layout and
    // Repaint.  This means that any state change must occur before Layout or
    // after Repaint.  This if statement implements the former.  It updates the
    // vessel and the editors to reflect the current state of the plugin and
    // then proceeds with the UI code.
    if (UnityEngine.Event.current.type == UnityEngine.EventType.Layout) {
      UpdateVesselAndBurnEditors();
    }

    // The UI code proper, executed identically for Layout and Repaint.  We
    // can freely change the state in events like clicks (e.g., in if statements
    // for buttons) as these don't happen between Layout and Repaint.
    string vessel_guid = predicted_vessel?.id.ToString();
    if (vessel_guid == null) {
      return;
    }

    using (new UnityEngine.GUILayout.HorizontalScope()) {
      int flight_plans = plugin.FlightPlanCount(vessel_guid);
      int selected_flight_plan = plugin.FlightPlanSelected(vessel_guid);
      for (int i = 0; i < flight_plans; ++i) {
        var id = new string(L10N.CacheFormat("#Principia_AlphabeticList")[i],
                            1);
        if (UnityEngine.GUILayout.Toggle(i == selected_flight_plan,
                                         id,
                                         "Button",
                                         GUILayoutWidth(1)) &&
            i != selected_flight_plan) {
          plugin.FlightPlanSelect(vessel_guid, i);
          final_time_.value_if_different =
              plugin.FlightPlanGetDesiredFinalTime(vessel_guid);
          ClearBurnEditors();
          UpdateVesselAndBurnEditors();
        }
      }
      bool must_create_flight_plan = false;
      if (flight_plans == 0) {
        must_create_flight_plan = UnityEngine.GUILayout.Button(
            L10N.CacheFormat("#Principia_FlightPlan_Create"));
      } else if (flight_plans < max_flight_plans) {
        must_create_flight_plan =
            UnityEngine.GUILayout.Button("+", GUILayoutWidth(1));
      }
      if (must_create_flight_plan) {
        plugin.FlightPlanCreate(vessel_guid,
                                plugin.CurrentTime() + 3600,
                                predicted_vessel.GetTotalMass());
        final_time_.value_if_different =
            plugin.FlightPlanGetDesiredFinalTime(vessel_guid);
        ClearBurnEditors();
        UpdateVesselAndBurnEditors();
        return;
      }
    }

    if (plugin.FlightPlanExists(vessel_guid)) {
      RenderFlightPlan(vessel_guid);
    }
    UnityEngine.GUI.DragWindow();
  }

  private void ClearBurnEditors() {
    if (burn_editors_ != null) {
      foreach (BurnEditor editor in burn_editors_) {
        editor.Close();
      }
      burn_editors_ = null;
      ScheduleShrink();
    }
  }

  private void UpdateVesselAndBurnEditors() {
    {
      string vessel_guid = predicted_vessel?.id.ToString();
      if (vessel_guid == null ||
          previous_predicted_vessel_ != predicted_vessel ||
          !plugin.FlightPlanExists(vessel_guid) ||
          plugin.FlightPlanNumberOfManoeuvres(vessel_guid) !=
          burn_editors_?.Count) {
        ClearBurnEditors();
        previous_predicted_vessel_ = predicted_vessel;
      }

      if (vessel_guid != null && plugin.FlightPlanExists(vessel_guid)) {
        bool changed = plugin.FlightPlanUpdateFromOptimization(vessel_guid);
        if (changed) {
          for (int i = 0; i < (burn_editors_?.Count ?? 0); ++i) {
            if (burn_editors_[i] is BurnEditor editor) {
              editor.Reset(plugin.FlightPlanGetManoeuvre(vessel_guid, i));
            }
          }
        }
      }
    }

    if (burn_editors_ == null) {
      string vessel_guid = predicted_vessel?.id.ToString();
      if (vessel_guid != null &&
          plugin.FlightPlanExists(vessel_guid)) {
        burn_editors_ = new List<BurnEditor>();
        final_time_.value_if_different =
            plugin.FlightPlanGetDesiredFinalTime(vessel_guid);
        for (int i = 0;
             i < plugin.FlightPlanNumberOfManoeuvres(vessel_guid);
             ++i) {
          // Dummy initial time, we call `Reset` immediately afterwards.
          burn_editors_.Add(new BurnEditor(adapter_,
                                           predicted_vessel,
                                           initial_time      : 0,
                                           get_burn_at_index : burn_editors_.
                                               ElementAtOrDefault));
          burn_editors_.Last().Reset(
              plugin.FlightPlanGetManoeuvre(vessel_guid, i));
        }
        UpdateBurnEditorIndices(vessel_guid);
      }
    }

    if (burn_editors_ != null) {
      string vessel_guid = predicted_vessel?.id.ToString();
      double current_time = plugin.CurrentTime();
      first_future_manœuvre_ = null;
      for (int i = 0; i < burn_editors_.Count; ++i) {
        NavigationManoeuvre manœuvre =
            plugin.FlightPlanGetManoeuvre(vessel_guid, i);
        if (current_time < manœuvre.final_time) {
          first_future_manœuvre_ = i;
          break;
        }
      }

      // Must be computed during layout as it affects the layout of some of the
      // differential sliders.
      number_of_anomalous_manœuvres_ =
          plugin.FlightPlanNumberOfAnomalousManoeuvres(vessel_guid);

      // Detect if the anomalous status has changed asynchronously because of
      // the prolongator.  If so, we will "tickle", i.e., pretend that the
      // desired final time changed.
      bool reached_deadline = plugin.FlightPlanGetAnomalousStatus(vessel_guid).
          is_deadline_exceeded();
      must_tickle_ = reached_deadline != reached_deadline_;
    }
  }

  private void RenderFlightPlan(string vessel_guid) {
    using (new UnityEngine.GUILayout.VerticalScope()) {
      RenderTabBar();
      Style.HorizontalLine();

      if (selected_tab_ == FlightPlannerTab.Optimizer) {
        RenderOptimizerTab(vessel_guid);
        return;
      }
      if (selected_tab_ == FlightPlannerTab.Transfer) {
        RenderTransferTab(vessel_guid);
        return;
      }
      if (selected_tab_ == FlightPlannerTab.Timeline) {
        RenderTimelineTab(vessel_guid);
        return;
      }
      if (selected_tab_ == FlightPlannerTab.ImportExport) {
        RenderImportExportTab(vessel_guid);
        return;
      }

      // A change of anomalous status "tickles" the flight plan.  Note that the
      // order of the terms in the || below matters, we always want to render
      // the `final_time_`.
      if (final_time_.Render(enabled : true) || must_tickle_) {
        must_tickle_ = false;
        var status =
            plugin.FlightPlanSetDesiredFinalTime(
                vessel_guid,
                final_time_.value);
        reached_deadline_ = status.is_deadline_exceeded();
        UpdateStatus(status, null);
        ResetOptimizer(vessel_guid);
      }
      // Always refresh the final time from C++ as it may have changed because
      // the last burn changed.
      final_time_.value_if_different =
          plugin.FlightPlanGetDesiredFinalTime(vessel_guid);

      FlightPlanAdaptiveStepParameters parameters =
          plugin.FlightPlanGetAdaptiveStepParameters(vessel_guid);
      length_integration_tolerance_index_ = Math.Max(
          0,
          Array.FindIndex(integration_tolerances_,
                          (double tolerance) => tolerance >=
                                                parameters.
                                                    length_integration_tolerance));
      speed_integration_tolerance_index_ = Math.Max(
          0,
          Array.FindIndex(integration_tolerances_,
                          (double tolerance) => tolerance >=
                                                parameters.
                                                    speed_integration_tolerance));
      max_steps_index_ = Math.Max(0,
                                  Array.FindIndex(
                                      max_steps_,
                                      (long step) =>
                                          step >= parameters.max_steps));

      using (new UnityEngine.GUILayout.HorizontalScope()) {
        using (new UnityEngine.GUILayout.HorizontalScope()) {
          UnityEngine.GUILayout.Label(
              L10N.CacheFormat("#Principia_FlightPlan_MaxSteps"),
              GUILayoutWidth(6));
          if (max_steps_index_ == 0) {
            UnityEngine.GUILayout.Button(
                L10N.CacheFormat("#Principia_DiscreteSelector_Min"));
          } else if (UnityEngine.GUILayout.Button("−")) {
            --max_steps_index_;
            UpdateAdaptiveStepParameters(ref parameters);
            var status =
                plugin.FlightPlanSetAdaptiveStepParameters(
                    vessel_guid,
                    parameters);
            UpdateStatus(status, null);
            ResetOptimizer(vessel_guid);
          }
          UnityEngine.GUILayout.TextArea(
              max_steps_[max_steps_index_].ToString(),
              GUILayoutWidth(3));
          if (max_steps_index_ == max_steps_.Length - 1) {
            UnityEngine.GUILayout.Button(
                L10N.CacheFormat("#Principia_DiscreteSelector_Max"));
          } else if (UnityEngine.GUILayout.Button("+")) {
            ++max_steps_index_;
            UpdateAdaptiveStepParameters(ref parameters);
            var status =
                plugin.FlightPlanSetAdaptiveStepParameters(
                    vessel_guid,
                    parameters);
            UpdateStatus(status, null);
            ResetOptimizer(vessel_guid);
          }
        }
        using (new UnityEngine.GUILayout.HorizontalScope()) {
          UnityEngine.GUILayout.Label(
              L10N.CacheFormat("#Principia_PredictionSettings_ToleranceLabel"),
              GUILayoutWidth(3));
          // Prior to Ἵππαρχος the tolerances were powers of 2, see #3395.
          if (length_integration_tolerance_index_ == 0 ||
              speed_integration_tolerance_index_ == 0) {
            UnityEngine.GUILayout.Button(
                L10N.CacheFormat("#Principia_DiscreteSelector_Min"));
          } else if (UnityEngine.GUILayout.Button("−")) {
            --length_integration_tolerance_index_;
            --speed_integration_tolerance_index_;
            UpdateAdaptiveStepParameters(ref parameters);
            var status =
                plugin.FlightPlanSetAdaptiveStepParameters(
                    vessel_guid,
                    parameters);
            UpdateStatus(status, null);
            ResetOptimizer(vessel_guid);
          }
          UnityEngine.GUILayout.TextArea(
              length_integration_tolerances_names_[
                  length_integration_tolerance_index_],
              GUILayoutWidth(3));
          if (length_integration_tolerance_index_ ==
              integration_tolerances_.Length - 1 ||
              speed_integration_tolerance_index_ ==
              integration_tolerances_.Length - 1) {
            UnityEngine.GUILayout.Button(
                L10N.CacheFormat("#Principia_DiscreteSelector_Max"));
          } else if (UnityEngine.GUILayout.Button("+")) {
            ++length_integration_tolerance_index_;
            ++speed_integration_tolerance_index_;
            UpdateAdaptiveStepParameters(ref parameters);
            var status =
                plugin.FlightPlanSetAdaptiveStepParameters(
                    vessel_guid,
                    parameters);
            UpdateStatus(status, null);
            ResetOptimizer(vessel_guid);
          }
        }
      }

      double Δv = (from burn_editor in burn_editors_ select burn_editor.Δv()).
          Sum();
      UnityEngine.GUILayout.Label(L10N.CacheFormat(
                                      "#Principia_FlightPlan_TotalΔv",
                                      Δv.ToString("0.000")));

      {
        var style = Style.Warning(Style.Multiline(UnityEngine.GUI.skin.label));
        string message = GetStatusMessage();
        // Size the label explicitly so that it doesn't decrease when the
        // message goes away: that causes annoying flicker.  The enclosing
        // window has a width of 20 units, but not all of that is available,
        // hence 19.
        warning_height_ = Math.Max(warning_height_,
                                   style.CalcHeight(
                                       new UnityEngine.GUIContent(message),
                                       Width(19)));
        UnityEngine.GUILayout.Label(message,
                                    style,
                                    UnityEngine.GUILayout.Height(
                                        warning_height_));
      }

      if (burn_editors_.Count == 0 &&
          UnityEngine.GUILayout.Button(
              L10N.CacheFormat("#Principia_FlightPlan_Delete"))) {
        final_trajectory_analyser_.DisposeWindow();
        final_trajectory_analyser_ =
            new PlannedOrbitAnalyser(adapter_, predicted_vessel_);
        plugin.FlightPlanDelete(vessel_guid);
        ResetStatus();
        ScheduleShrink();
        // The state change will happen the next time we go through OnGUI.
      } else {
        using (new UnityEngine.GUILayout.HorizontalScope()) {
          if (UnityEngine.GUILayout.Button(
                  L10N.CacheFormat("#Principia_FlightPlan_Rebase"))) {
            var status = plugin.FlightPlanRebase(
                vessel_guid,
                predicted_vessel.GetTotalMass());
            UpdateStatus(status, null);
            ResetOptimizer(vessel_guid);
            if (status.ok()) {
              // The final time does not change, but since it is displayed with
              // respect to the beginning of the flight plan, the text must be
              // recomputed.
              final_time_.value =
                  plugin.FlightPlanGetDesiredFinalTime(vessel_guid);
              UpdateBurnEditorIndices(vessel_guid);
              return;
            }
          }
          if (plugin.FlightPlanCount(vessel_guid) < max_flight_plans &&
              UnityEngine.GUILayout.Button(
                  L10N.CacheFormat("#Principia_FlightPlan_Duplicate"))) {
            plugin.FlightPlanDuplicate(vessel_guid);
          }
        }

        // There is no Layout/Repaint trouble here because the frame is selected
        // in another window.
        if (adapter_.plotting_frame_selector_.
                Centre() is CelestialBody centre) {
          Style.HorizontalLine();
          using (new UnityEngine.GUILayout.HorizontalScope()) {
            UnityEngine.GUILayout.Label(
                L10N.CelestialString("#Principia_FlightPlan_Optimization",
                                     new[]{ centre }),
                style: Style.MiddleLeftAligned(UnityEngine.GUI.skin.label,
                                               Height(2)));
            using (new UnityEngine.GUILayout.VerticalScope()) {
              double optimization_altitude = optimization_altitude_;
              double? optimization_inclination_in_degrees =
                  optimization_inclination_in_degrees_;

              using (new UnityEngine.GUILayout.HorizontalScope()) {
                UnityEngine.GUILayout.Label(
                    L10N.CacheFormat("#Principia_FlightPlan_TargetAltitude"));
                string text = UnityEngine.GUILayout.TextField(
                    optimization_altitude.FormatN(0),
                    GUILayoutWidth(3));
                UnityEngine.GUILayout.Label(
                    text    : L10N.CacheFormat(
                        "#Principia_FlightPlan_AltitudeUnit"),
                    options : GUILayoutWidth(1));
                UnityEngine.GUILayout.Label(
                    text    : "",
                    options : GUILayoutWidth(2));
                if (double.TryParse(text,
                                    System.Globalization.NumberStyles.Any,
                                    Culture.culture,
                                    out double candidate)) {
                  if (candidate >= 0 && candidate < double.PositiveInfinity) {
                    optimization_altitude = candidate;
                  }
                }
              }

              using (new UnityEngine.GUILayout.HorizontalScope()) {
                UnityEngine.GUILayout.Label(
                    L10N.CacheFormat("#Principia_FlightPlan_TargetInclination"));
                string text = UnityEngine.GUILayout.TextField(
                    optimization_inclination_in_degrees.HasValue
                        ? optimization_inclination_in_degrees.Value.FormatN(0)
                        : L10N.CacheFormat(
                            "#Principia_FlightPlan_OptimizeInclinationNoText"),
                    GUILayoutWidth(3));
                UnityEngine.GUILayout.Label(
                    text: L10N.CacheFormat(
                        "#Principia_FlightPlan_InclinationUnit"),
                    options: GUILayoutWidth(1));
                bool optimize_inclination = UnityEngine.GUILayout.Toggle(
                        optimization_inclination_in_degrees.HasValue,
                        optimization_inclination_in_degrees.HasValue
                            ? L10N.CacheFormat(
                                "#Principia_FlightPlan_OptimizeInclinationOn")
                            : L10N.CacheFormat(
                                "#Principia_FlightPlan_OptimizeInclinationOff"),
                        GUILayoutWidth(2));
                if (!optimize_inclination) {
                  optimization_inclination_in_degrees = null;
                } else if (text ==
                           L10N.CacheFormat(
                               "#Principia_FlightPlan_OptimizeInclinationNoText")) {
                  optimization_inclination_in_degrees = 0;
                } else if (double.TryParse(text,
                                           System.Globalization.NumberStyles.
                                               Any,
                                           Culture.culture,
                                           out double candidate)) {
                  optimization_inclination_in_degrees =
                      Math.Max(Math.Min(180, candidate), -180);
                }
              }

              // If any of the parameters changed (that includes a change of
              // plotting frame in another window), recreate the optimization
              // driver.  This interrupts any optimization that might be
              // running, to avoid confusing results.
              if (optimization_altitude_ != optimization_altitude ||
                  optimization_inclination_in_degrees_ !=
                  optimization_inclination_in_degrees ||
                  optimization_reference_frame_parameters_ !=
                  (NavigationFrameParameters)adapter_.plotting_frame_selector_.
                      FrameParameters()) {
                ResetOptimizer(vessel_guid,
                               centre,
                               optimization_altitude,
                               optimization_inclination_in_degrees);
              }
            }
          }
        }

        if (burn_editors_.Count > 0) {
          RenderUpcomingEvents();
        }

        if (requested_editor_focus_index_ is int requested_focus) {
          requested_editor_focus_index_ = null;
          for (int i = 0; i < burn_editors_.Count; ++i) {
            burn_editors_[i].minimized = requested_focus != i;
          }
          ScheduleShrink();
        }

        // Compute the final times for each manœuvre before displaying them.
        var final_times = new List<double>();
        for (int i = 0; i < burn_editors_.Count - 1; ++i) {
          final_times.Add(plugin.FlightPlanGetManoeuvre(vessel_guid, i + 1).
                              burn.initial_time);
        }
        // Allow extending the flight plan.
        final_times.Add(double.PositiveInfinity);

        for (int i = 0; i < burn_editors_.Count; ++i) {
          Style.HorizontalLine();
          if (RenderCoast(i, out double? orbital_period)) {
            return;
          }
          using (new UnityEngine.GUILayout.HorizontalScope()) {
            UnityEngine.GUILayout.Label(
                L10N.CacheFormat("#Principia_FlightPlan_ManœuvreHeader",
                                 i + 1));
            if (i == 0) {
              UnityEngine.GUILayout.Button("↑", GUILayoutWidth(1));
            } else if (UnityEngine.GUILayout.Button("↑", GUILayoutWidth(1))) {
              MoveManœuvre(vessel_guid, i, i - 1);
              return;
            }
            if (i >= burn_editors_.Count - 1) {
              UnityEngine.GUILayout.Button("↓", GUILayoutWidth(1));
            } else if (UnityEngine.GUILayout.Button("↓", GUILayoutWidth(1))) {
              MoveManœuvre(vessel_guid, i, i + 1);
              return;
            }
          }
          Style.HorizontalLine();
          BurnEditor burn = burn_editors_[i];
          switch (burn.Render(
                      header          :
                      L10N.CacheFormat("#Principia_FlightPlan_ManœuvreHeader",
                                       i + 1),
                      anomalous       : i >=
                                        burn_editors_.Count -
                                        number_of_anomalous_manœuvres_,
                      burn_final_time : final_times[i],
                      orbital_period  : orbital_period)) {
            case BurnEditor.Event.Deleted: {
              var status = plugin.FlightPlanRemove(vessel_guid, i);
              UpdateStatus(status, null);
              ResetOptimizer(vessel_guid);
              burn_editors_[i].Close();
              burn_editors_.RemoveAt(i);
              UpdateBurnEditorIndices(vessel_guid);
              ScheduleShrink();
              return;
            }
            case BurnEditor.Event.Minimized:
            case BurnEditor.Event.Maximized: {
              ScheduleShrink();
              return;
            }
            case BurnEditor.Event.Changed: {
              ReplaceBurn(i, burn.Burn());
              break;
            }
            case BurnEditor.Event.None: {
              break;
            }
          }
        }
        Style.HorizontalLine();
        if (RenderCoast(burn_editors_.Count, orbital_period: out _)) {
          return;
        }
      }
    }
  }

  private void RenderTabBar() {
    using (new UnityEngine.GUILayout.HorizontalScope()) {
      RenderTabButton(FlightPlannerTab.Planner, "Planner");
      RenderTabButton(FlightPlannerTab.Optimizer, "Optimizer");
      RenderTabButton(FlightPlannerTab.Transfer, "Transfer");
      RenderTabButton(FlightPlannerTab.Timeline, "Timeline");
      RenderTabButton(FlightPlannerTab.ImportExport, "Import/Export");
    }
  }

  private void RenderTabButton(FlightPlannerTab tab, string label) {
    if (UnityEngine.GUILayout.Toggle(
            selected_tab_ == tab,
            label,
            "Button",
            GUILayoutWidth(3))) {
      selected_tab_ = tab;
    }
  }

  private void RenderOptimizerTab(string vessel_guid) {
    if (burn_editors_ == null || burn_editors_.Count == 0) {
      UnityEngine.GUILayout.Label("Add at least one manoeuvre to optimize.");
      return;
    }

    int in_progress =
        plugin.FlightPlanOptimizationDriverInProgress(vessel_guid);
    UnityEngine.GUILayout.Label(
        in_progress >= 0
            ? "Optimizer status: running on manoeuvre #" + (in_progress + 1)
            : "Optimizer status: idle");

    if (adapter_.plotting_frame_selector_.Centre() is CelestialBody centre) {
      using (new UnityEngine.GUILayout.HorizontalScope()) {
        UnityEngine.GUILayout.Label("Target altitude");
        string altitude_text = UnityEngine.GUILayout.TextField(
            optimization_altitude_.FormatN(0),
            GUILayoutWidth(4));
        UnityEngine.GUILayout.Label("m");
        if (double.TryParse(altitude_text,
                            System.Globalization.NumberStyles.Any,
                            Culture.culture,
                            out double altitude_candidate) &&
            altitude_candidate >= 0) {
          optimization_altitude_ = altitude_candidate;
        }
      }

      using (new UnityEngine.GUILayout.HorizontalScope()) {
        UnityEngine.GUILayout.Label("Target inclination");
        string inclination_text = UnityEngine.GUILayout.TextField(
            optimization_inclination_in_degrees_.HasValue
                ? optimization_inclination_in_degrees_.Value.FormatN(0)
                : "Any",
            GUILayoutWidth(4));
        UnityEngine.GUILayout.Label("deg");
        bool optimize_inclination = UnityEngine.GUILayout.Toggle(
            optimization_inclination_in_degrees_.HasValue,
            optimization_inclination_in_degrees_.HasValue ? "Constrained"
                                                          : "Unconstrained");
        if (!optimize_inclination) {
          optimization_inclination_in_degrees_ = null;
        } else if (double.TryParse(inclination_text,
                                   System.Globalization.NumberStyles.Any,
                                   Culture.culture,
                                   out double inclination_candidate)) {
          optimization_inclination_in_degrees_ = Math.Max(
              Math.Min(180, inclination_candidate),
              -180);
        }
      }

      using (new UnityEngine.GUILayout.HorizontalScope()) {
        UnityEngine.GUILayout.Label("Selected manoeuvre");
        if (selected_optimization_manœuvre_ > 0 &&
            UnityEngine.GUILayout.Button("−", GUILayoutWidth(1))) {
          --selected_optimization_manœuvre_;
        }
        UnityEngine.GUILayout.TextArea(
            (selected_optimization_manœuvre_ + 1).ToString(),
            GUILayoutWidth(2));
        if (selected_optimization_manœuvre_ < burn_editors_.Count - 1 &&
            UnityEngine.GUILayout.Button("+", GUILayoutWidth(1))) {
          ++selected_optimization_manœuvre_;
        }
      }

      using (new UnityEngine.GUILayout.HorizontalScope()) {
        if (UnityEngine.GUILayout.Button("Rebuild optimizer")) {
          ResetOptimizer(vessel_guid,
                         centre,
                         optimization_altitude_,
                         optimization_inclination_in_degrees_);
        }
        if (UnityEngine.GUILayout.Button("Optimize selected") &&
            in_progress == -1) {
          plugin.FlightPlanOptimizationDriverStart(vessel_guid,
                                                   selected_optimization_manœuvre_);
        }
      }
    } else {
      UnityEngine.GUILayout.Label(
          "Switch to a body-centred plotting frame to use optimization.");
    }
  }

  private void RenderTransferTab(string vessel_guid) {
    InitializeTransferBodies();

    using (new UnityEngine.GUILayout.HorizontalScope()) {
      UnityEngine.GUILayout.Label("Origin");
      if (UnityEngine.GUILayout.Button("<", GUILayoutWidth(1))) {
        CycleTransferBody(is_origin : true, direction : -1);
      }
      UnityEngine.GUILayout.TextArea(
          transfer_origin_?.bodyName ?? "(none)",
          GUILayoutWidth(5));
      if (UnityEngine.GUILayout.Button(">", GUILayoutWidth(1))) {
        CycleTransferBody(is_origin : true, direction : 1);
      }
    }

    using (new UnityEngine.GUILayout.HorizontalScope()) {
      UnityEngine.GUILayout.Label("Target");
      if (UnityEngine.GUILayout.Button("<", GUILayoutWidth(1))) {
        CycleTransferBody(is_origin : false, direction : -1);
      }
      UnityEngine.GUILayout.TextArea(
          transfer_target_?.bodyName ?? "(none)",
          GUILayoutWidth(5));
      if (UnityEngine.GUILayout.Button(">", GUILayoutWidth(1))) {
        CycleTransferBody(is_origin : false, direction : 1);
      }
    }

    if (!TryComputeHohmannTransfer(out TransferEstimate estimate,
                                   out string issue)) {
      UnityEngine.GUILayout.Label(issue, Style.Warning(UnityEngine.GUI.skin.label));
      return;
    }

    UnityEngine.GUILayout.Label("Transfer summary");
    UnityEngine.GUILayout.Label(
        "Window in " + FormatPositiveTimeSpan(estimate.wait_time));
    UnityEngine.GUILayout.Label(
        "Transfer time " + FormatPositiveTimeSpan(estimate.transfer_time));
    UnityEngine.GUILayout.Label(
        "Ejection Δv " + estimate.ejection_Δv.ToString("0.000") + " m/s");
    UnityEngine.GUILayout.Label(
        "Capture Δv " + estimate.capture_Δv.ToString("0.000") + " m/s");

    using (new UnityEngine.GUILayout.HorizontalScope()) {
      if (UnityEngine.GUILayout.Button("Create transfer seed burn")) {
        CreateTransferSeedBurn(vessel_guid, estimate);
      }
      if (UnityEngine.GUILayout.Button("Copy summary")) {
        transfer_summary_ =
            transfer_origin_.bodyName + " -> " + transfer_target_.bodyName +
            ", window " + FormatPositiveTimeSpan(estimate.wait_time) +
            ", Δv " + estimate.ejection_Δv.ToString("0.000") + " m/s";
      }
    }
    UnityEngine.GUILayout.Label(transfer_summary_);
  }

  private void RenderTimelineTab(string vessel_guid) {
    if (!plugin.FlightPlanExists(vessel_guid)) {
      UnityEngine.GUILayout.Label("No active flight plan.");
      return;
    }

    double initial_time = plugin.FlightPlanGetInitialTime(vessel_guid);
    double final_time = plugin.FlightPlanGetDesiredFinalTime(vessel_guid);
    if (!timeline_initialized_) {
      timeline_time_ = plugin.CurrentTime();
      timeline_initialized_ = true;
    }
    timeline_time_ = Math.Max(initial_time, Math.Min(final_time, timeline_time_));

    UnityEngine.GUILayout.Label("Trajectory timeline");
    timeline_time_ = UnityEngine.GUILayout.HorizontalSlider(
        (float)timeline_time_,
        (float)initial_time,
        (float)final_time);
    UnityEngine.GUILayout.Label(
        "T + " + FormatPositiveTimeSpan(timeline_time_ - initial_time));
    UnityEngine.GUILayout.Label(
        "Absolute: " + FormatTimeSpan(timeline_time_));

    using (new UnityEngine.GUILayout.HorizontalScope()) {
      if (UnityEngine.GUILayout.Button("Warp to cursor")) {
        TimeWarp.fetch.WarpTo(timeline_time_);
      }
      if (UnityEngine.GUILayout.Button("Set plan end to cursor")) {
        var status = plugin.FlightPlanSetDesiredFinalTime(vessel_guid,
                                                          timeline_time_);
        UpdateStatus(status, null);
      }
    }

    UnityEngine.GUILayout.Label("Manoeuvre markers");
    for (int i = 0; i < burn_editors_.Count; ++i) {
      NavigationManoeuvre manœuvre = plugin.FlightPlanGetManoeuvre(vessel_guid,
                                                                    i);
      using (new UnityEngine.GUILayout.HorizontalScope()) {
        UnityEngine.GUILayout.Label(
            "#" + (i + 1) + " @ " +
            FormatPositiveTimeSpan(
                manœuvre.burn.initial_time - initial_time));
        if (UnityEngine.GUILayout.Button("Go", GUILayoutWidth(1.5f))) {
          timeline_time_ = manœuvre.burn.initial_time;
        }
      }
    }
  }

  private void RenderImportExportTab(string vessel_guid) {
    EnsurePlanDirectoryExists();
    RefreshPlanFiles();

    UnityEngine.GUILayout.Label("Flight plan file exchange");
    if (UnityEngine.GUILayout.Button("Export selected plan")) {
      ExportCurrentFlightPlan(vessel_guid);
    }

    using (new UnityEngine.GUILayout.HorizontalScope()) {
      if (available_plan_files_.Count == 0) {
        UnityEngine.GUILayout.Label("No saved plans found.");
      } else {
        if (UnityEngine.GUILayout.Button("<", GUILayoutWidth(1))) {
          selected_plan_file_index_ =
              (selected_plan_file_index_ - 1 + available_plan_files_.Count) %
              available_plan_files_.Count;
        }
        UnityEngine.GUILayout.TextArea(
            Path.GetFileName(available_plan_files_[selected_plan_file_index_]),
            GUILayoutWidth(8));
        if (UnityEngine.GUILayout.Button(">", GUILayoutWidth(1))) {
          selected_plan_file_index_ =
              (selected_plan_file_index_ + 1) % available_plan_files_.Count;
        }
      }
    }

    if (available_plan_files_.Count > 0 &&
        UnityEngine.GUILayout.Button("Import into selected flight plan slot")) {
      ImportFlightPlan(vessel_guid,
                       available_plan_files_[selected_plan_file_index_]);
    }
    UnityEngine.GUILayout.Label(io_status_message_);
  }

  private bool MoveManœuvre(string vessel_guid, int from, int to) {
    if (from < 0 || to < 0 || from >= burn_editors_.Count || to >= burn_editors_.Count) {
      return false;
    }
    NavigationManoeuvre moving = plugin.FlightPlanGetManoeuvre(vessel_guid,
                                                                from);
    var remove_status = plugin.FlightPlanRemove(vessel_guid, from);
    UpdateStatus(remove_status, from);
    if (!remove_status.ok()) {
      return false;
    }

    if (to > from) {
      --to;
    }
    var insert_status = plugin.FlightPlanInsert(vessel_guid,
                                                moving.burn,
                                                to);
    UpdateStatus(insert_status, to);
    if (!insert_status.ok()) {
      plugin.FlightPlanInsert(vessel_guid, moving.burn, from);
      return false;
    }

    BurnEditor editor = burn_editors_[from];
    burn_editors_.RemoveAt(from);
    burn_editors_.Insert(to, editor);
    UpdateBurnEditorIndices(vessel_guid);
    ResetOptimizer(vessel_guid);
    return true;
  }

  private void InitializeTransferBodies() {
    if (transfer_origin_ == null) {
      transfer_origin_ = FlightGlobals.currentMainBody ?? FlightGlobals.GetHomeBody();
    }
    if (transfer_target_ == null) {
      transfer_target_ = transfer_origin_.orbitingBodies.FirstOrDefault() ??
                         transfer_origin_.referenceBody ??
                         transfer_origin_;
    }
  }

  private void CycleTransferBody(bool is_origin, int direction) {
    InitializeTransferBodies();
    CelestialBody basis = is_origin ? transfer_origin_ : transfer_target_;
    CelestialBody parent = basis?.referenceBody ??
                           transfer_origin_?.referenceBody ??
                           FlightGlobals.GetHomeBody().referenceBody;
    List<CelestialBody> candidates = parent?.orbitingBodies?.ToList() ??
                                     FlightGlobals.Bodies.ToList();
    if (candidates.Count == 0) {
      return;
    }
    int index = candidates.IndexOf(basis);
    if (index < 0) {
      index = 0;
    }
    index = (index + direction + candidates.Count) % candidates.Count;
    if (is_origin) {
      transfer_origin_ = candidates[index];
    } else {
      transfer_target_ = candidates[index];
    }
  }

  private bool TryComputeHohmannTransfer(out TransferEstimate estimate,
                                         out string issue) {
    estimate = default;
    issue = null;
    if (transfer_origin_ == null || transfer_target_ == null) {
      issue = "Select both origin and target bodies.";
      return false;
    }
    if (transfer_origin_ == transfer_target_) {
      issue = "Origin and target must be different bodies.";
      return false;
    }
    if (transfer_origin_.referenceBody != transfer_target_.referenceBody ||
        transfer_origin_.orbit == null ||
        transfer_target_.orbit == null) {
      issue = "Hohmann planning requires two sibling orbiters of the same primary body.";
      return false;
    }

    CelestialBody primary = transfer_origin_.referenceBody;
    double μ = primary.gravParameter;
    double r1 = transfer_origin_.orbit.semiMajorAxis;
    double r2 = transfer_target_.orbit.semiMajorAxis;
    double a_transfer = 0.5 * (r1 + r2);
    double transfer_time = Math.PI * Math.Sqrt(a_transfer * a_transfer * a_transfer / μ);
    double v1 = Math.Sqrt(μ / r1);
    double v2 = Math.Sqrt(μ / r2);
    double v_transfer_periapsis = Math.Sqrt(μ * (2 / r1 - 1 / a_transfer));
    double v_transfer_apoapsis = Math.Sqrt(μ * (2 / r2 - 1 / a_transfer));
    double ejection_Δv = Math.Abs(v_transfer_periapsis - v1);
    double capture_Δv = Math.Abs(v2 - v_transfer_apoapsis);

    double n1 = 2 * Math.PI / transfer_origin_.orbit.period;
    double n2 = 2 * Math.PI / transfer_target_.orbit.period;
    double synodic_rate = Math.Abs(n1 - n2);
    if (synodic_rate <= 0) {
      issue = "Unable to compute a synodic period for these orbits.";
      return false;
    }

    double current_phase = NormalizeAngle(
        (transfer_target_.orbit.LAN + transfer_target_.orbit.argumentOfPeriapsis +
         transfer_target_.orbit.trueAnomaly) -
        (transfer_origin_.orbit.LAN + transfer_origin_.orbit.argumentOfPeriapsis +
         transfer_origin_.orbit.trueAnomaly));
    double required_phase = NormalizeAngle(Math.PI - n2 * transfer_time);
    double wait_time = NormalizeAngle(required_phase - current_phase) / synodic_rate;

    estimate = new TransferEstimate {
        wait_time = wait_time,
        transfer_time = transfer_time,
        ejection_Δv = ejection_Δv,
        capture_Δv = capture_Δv
    };
    return true;
  }

  private void CreateTransferSeedBurn(string vessel_guid, TransferEstimate estimate) {
    if (!plugin.FlightPlanExists(vessel_guid)) {
      plugin.FlightPlanCreate(vessel_guid,
                              plugin.CurrentTime() + 60,
                              predicted_vessel.GetTotalMass());
      ClearBurnEditors();
      UpdateVesselAndBurnEditors();
    }

    double initial_time = plugin.CurrentTime() + estimate.wait_time;
    var temporary_editor = new BurnEditor(adapter_,
                                          predicted_vessel,
                                          initial_time,
                                          get_burn_at_index : i => null);
    Burn seed = temporary_editor.Burn();
    temporary_editor.Close();
    seed.initial_time = initial_time;
    seed.delta_v = new XYZ { x = estimate.ejection_Δv, y = 0, z = 0 };
    var status = plugin.FlightPlanInsert(vessel_guid,
                                         seed,
                                         plugin.FlightPlanNumberOfManoeuvres(vessel_guid));
    UpdateStatus(status, null);
    if (status.ok()) {
      io_status_message_ = "Transfer seed burn added to the end of the plan.";
      ClearBurnEditors();
      UpdateVesselAndBurnEditors();
    }
  }

  private static double NormalizeAngle(double angle) {
    double two_π = 2 * Math.PI;
    double normalized = angle % two_π;
    if (normalized < 0) {
      normalized += two_π;
    }
    return normalized;
  }

  private void EnsurePlanDirectoryExists() {
    if (!Directory.Exists(plan_exchange_directory_)) {
      Directory.CreateDirectory(plan_exchange_directory_);
    }
  }

  private void RefreshPlanFiles() {
    EnsurePlanDirectoryExists();
    available_plan_files_ = Directory.GetFiles(plan_exchange_directory_, "*.cfg")
        .OrderBy(path => path)
        .ToList();
    if (selected_plan_file_index_ >= available_plan_files_.Count) {
      selected_plan_file_index_ = Math.Max(0, available_plan_files_.Count - 1);
    }
  }

  private void ExportCurrentFlightPlan(string vessel_guid) {
    if (!plugin.FlightPlanExists(vessel_guid)) {
      io_status_message_ = "No active flight plan to export.";
      return;
    }

    var root = new ConfigNode("PRINCIPIA_FLIGHT_PLAN");
    root.AddValue("vessel_guid", vessel_guid);
    root.AddValue("vessel_name", predicted_vessel.vesselName);
    root.AddValue("initial_time", plugin.FlightPlanGetInitialTime(vessel_guid));
    root.AddValue("desired_final_time", plugin.FlightPlanGetDesiredFinalTime(vessel_guid));
    int manoeuvres = plugin.FlightPlanNumberOfManoeuvres(vessel_guid);
    for (int i = 0; i < manoeuvres; ++i) {
      NavigationManoeuvre manœuvre = plugin.FlightPlanGetManoeuvre(vessel_guid, i);
      Burn burn = manœuvre.burn;
      ConfigNode node = root.AddNode("MANOEUVRE");
      node.AddValue("initial_time", burn.initial_time);
      node.AddValue("thrust_in_kilonewtons", burn.thrust_in_kilonewtons);
      node.AddValue("specific_impulse_in_seconds_g0",
                    burn.specific_impulse_in_seconds_g0);
      node.AddValue("is_inertially_fixed", burn.is_inertially_fixed);
      node.AddValue("delta_v_x", burn.delta_v.x);
      node.AddValue("delta_v_y", burn.delta_v.y);
      node.AddValue("delta_v_z", burn.delta_v.z);
      node.AddValue("frame_extension", (int)burn.frame.Extension);
      node.AddValue("frame_centre", burn.frame.CentreIndex);
      node.AddValue("frame_primary", string.Join(",", burn.frame.PrimaryIndices));
      node.AddValue("frame_secondary", string.Join(",", burn.frame.SecondaryIndices));
    }

    string safe_name = string.Concat(
        (predicted_vessel.vesselName ?? "vessel").Select(
            c => Path.GetInvalidFileNameChars().Contains(c) ? '_' : c));
    string path = Path.Combine(plan_exchange_directory_,
                               safe_name + "_" +
                               DateTime.UtcNow.ToString("yyyyMMdd_HHmmss") +
                               ".cfg");
    root.Save(path);
    io_status_message_ = "Exported flight plan to " + Path.GetFileName(path);
    RefreshPlanFiles();
  }

  private void ImportFlightPlan(string vessel_guid, string path) {
    ConfigNode root = ConfigNode.Load(path);
    if (root == null) {
      io_status_message_ = "Import failed: could not read file.";
      return;
    }

    if (plugin.FlightPlanExists(vessel_guid)) {
      plugin.FlightPlanDelete(vessel_guid);
    }

    double initial_time =
        double.Parse(root.GetUniqueValue("initial_time"), Culture.culture);
    plugin.FlightPlanCreate(vessel_guid,
                            initial_time,
                            predicted_vessel.GetTotalMass());

    ConfigNode[] manoeuvre_nodes = root.GetNodes("MANOEUVRE");
    for (int i = 0; i < manoeuvre_nodes.Length; ++i) {
      ConfigNode node = manoeuvre_nodes[i];
      Burn burn = new Burn {
          initial_time = double.Parse(node.GetUniqueValue("initial_time"),
                                      Culture.culture),
          thrust_in_kilonewtons =
              double.Parse(node.GetUniqueValue("thrust_in_kilonewtons"),
                           Culture.culture),
          specific_impulse_in_seconds_g0 =
              double.Parse(node.GetUniqueValue("specific_impulse_in_seconds_g0"),
                           Culture.culture),
          is_inertially_fixed = bool.Parse(node.GetUniqueValue("is_inertially_fixed")),
          delta_v = new XYZ {
              x = double.Parse(node.GetUniqueValue("delta_v_x"), Culture.culture),
              y = double.Parse(node.GetUniqueValue("delta_v_y"), Culture.culture),
              z = double.Parse(node.GetUniqueValue("delta_v_z"), Culture.culture)
          },
          frame = new NavigationFrameParameters {
              Extension = (FrameType)int.Parse(node.GetUniqueValue("frame_extension")),
              CentreIndex = int.Parse(node.GetUniqueValue("frame_centre")),
              PrimaryIndices = ParseIndices(node.GetAtMostOneValue("frame_primary")),
              SecondaryIndices = ParseIndices(node.GetAtMostOneValue("frame_secondary"))
          }
      };

      var status = plugin.FlightPlanInsert(vessel_guid, burn, i);
      UpdateStatus(status, i);
      if (!status.ok()) {
        io_status_message_ = "Import failed while inserting manoeuvre #" + (i + 1);
        return;
      }
    }

    string desired_final_time_text = root.GetAtMostOneValue("desired_final_time");
    if (desired_final_time_text != null) {
      plugin.FlightPlanSetDesiredFinalTime(
          vessel_guid,
          double.Parse(desired_final_time_text, Culture.culture));
    }

    ClearBurnEditors();
    UpdateVesselAndBurnEditors();
    io_status_message_ = "Imported " + manoeuvre_nodes.Length + " manoeuvre(s).";
  }

  private static int[] ParseIndices(string csv) {
    if (string.IsNullOrWhiteSpace(csv)) {
      return new int[0];
    }
    return csv.Split(',')
              .Select(s => s.Trim())
              .Where(s => s.Length > 0)
              .Select(int.Parse)
              .ToArray();
  }

  private void RenderUpcomingEvents() {
    string vessel_guid = predicted_vessel.id.ToString();
    double current_time = plugin.CurrentTime();

    Style.HorizontalLine();
    if (first_future_manœuvre_.HasValue) {
      int first_future_manœuvre = first_future_manœuvre_.Value;
      NavigationManoeuvre manœuvre =
          plugin.FlightPlanGetManoeuvre(vessel_guid, first_future_manœuvre);
      if (manœuvre.burn.initial_time > current_time) {
        using (new UnityEngine.GUILayout.HorizontalScope()) {
          UnityEngine.GUILayout.Label(
              L10N.CacheFormat("#Principia_FlightPlan_UpcomingManœuvre",
                               first_future_manœuvre + 1));
          UnityEngine.GUILayout.Label(
              L10N.CacheFormat("#Principia_FlightPlan_IgnitionCountdown",
                               FormatTimeSpan(
                                   current_time - manœuvre.burn.initial_time)),
              style : Style.RightAligned(UnityEngine.GUI.skin.label));
        }
      } else {
        using (new UnityEngine.GUILayout.HorizontalScope()) {
          UnityEngine.GUILayout.Label(
              L10N.CacheFormat("#Principia_FlightPlan_OngoingManœuvre",
                               first_future_manœuvre + 1));
          UnityEngine.GUILayout.Label(
              L10N.CacheFormat("#Principia_FlightPlan_CutoffCountdown",
                               FormatTimeSpan(
                                   current_time - manœuvre.final_time)),
              style : Style.RightAligned(UnityEngine.GUI.skin.label));
        }
      }
      // In career mode, the patched conic solver may be null.  In that case
      // we do not offer the option of showing the manœuvre on the navball,
      // even though the flight planner is still available to plan it.
      // TODO(egg): We may want to consider setting the burn vector directly
      // rather than going through the solver.
      if (predicted_vessel.patchedConicSolver != null) {
        using (new UnityEngine.GUILayout.HorizontalScope()) {
          show_guidance_ = UnityEngine.GUILayout.Toggle(
              show_guidance_,
              L10N.CacheFormat("#Principia_FlightPlan_ShowManœuvreOnNavball"));
          if (UnityEngine.GUILayout.Button(
              L10N.CacheFormat("#Principia_FlightPlan_WarpToManœuvre"))) {
            TimeWarp.fetch.WarpTo(manœuvre.burn.initial_time - 60);
          }
        }
      }
    } else {
      // Reserve some space to avoid the UI changing shape if we have
      // nothing to say.
      UnityEngine.GUILayout.Label(
          L10N.CacheFormat(
              "#Principia_FlightPlan_Warning_AllManœuvresInThePast"),
          Style.Warning(UnityEngine.GUI.skin.label));
      UnityEngine.GUILayout.Space(Width(1));
    }
  }

  private bool RenderCoast(int index, out double? orbital_period) {
    string vessel_guid = predicted_vessel.id.ToString();
    var coast_analysis = plugin.FlightPlanGetCoastAnalysis(
        vessel_guid,
        revolutions_per_cycle   : null,
        days_per_cycle          : null,
        ground_track_revolution : 0,
        index);
    string orbit_description = null;
    orbital_period = coast_analysis.elements?.nodal_period;
    if (coast_analysis.primary_index.HasValue) {
      var primary = FlightGlobals.Bodies[coast_analysis.primary_index.Value];
      int? nodal_revolutions = (int?)(coast_analysis.mission_duration /
                                      coast_analysis.elements?.nodal_period);
      orbit_description = OrbitAnalyser.OrbitDescription(
          primary,
          coast_analysis.mission_duration,
          coast_analysis.elements,
          coast_analysis.recurrence,
          coast_analysis.ground_track_equatorial_crossings,
          coast_analysis.solar_times_of_nodes,
          nodal_revolutions);
    }
    using (new UnityEngine.GUILayout.HorizontalScope()) {
      if (index == burn_editors_.Count) {
        final_trajectory_analyser_.index = index;
        final_trajectory_analyser_.RenderButton();
      } else {
        double start_of_coast = index == 0
                                    ? plugin.FlightPlanGetInitialTime(
                                        vessel_guid)
                                    : burn_editors_[index - 1].final_time;
        string coast_duration =
            (burn_editors_[index].initial_time - start_of_coast).FormatDuration(
                show_seconds: false);
        string coast_description = orbit_description == null
                                       ? L10N.CacheFormat(
                                           "#Principia_FlightPlan_Coast",
                                           coast_duration)
                                       : L10N.CacheFormat(
                                           "#Principia_FlightPlan_CoastInOrbit",
                                           orbit_description,
                                           coast_duration);
        UnityEngine.GUILayout.Label(coast_description);
      }
      if (UnityEngine.GUILayout.Button(
              L10N.CacheFormat("#Principia_FlightPlan_AddManœuvre"),
              GUILayoutWidth(5))) {
        double initial_time;
        if (index == 0) {
          initial_time = plugin.CurrentTime() + 60;
        } else {
          initial_time =
              plugin.FlightPlanGetManoeuvre(vessel_guid,
                                            index - 1).final_time + 60;
        }
        var editor = new BurnEditor(adapter_,
                                    predicted_vessel,
                                    initial_time,
                                    get_burn_at_index : burn_editors_.
                                        ElementAtOrDefault){
            minimized = false
        };
        Burn candidate_burn = editor.Burn();
        var status = plugin.FlightPlanInsert(vessel_guid,
                                             candidate_burn,
                                             index);
        ResetOptimizer(vessel_guid);

        // The previous call did not necessarily create a manœuvre.  Check if
        // we need to add an editor.
        int number_of_manœuvres =
            plugin.FlightPlanNumberOfManoeuvres(vessel_guid);
        if (number_of_manœuvres > burn_editors_.Count) {
          editor.Reset(plugin.FlightPlanGetManoeuvre(vessel_guid, index));
          burn_editors_.Insert(index, editor);
          UpdateBurnEditorIndices(vessel_guid);
          UpdateStatus(status, index);
          ScheduleShrink();
          return true;
        }
        // TODO(phl): The error messaging here will be either confusing or
        // wrong.  The messages should mention the new manœuvre without
        // numbering it, since the numbering has not changed (“the new manœuvre
        // would overlap with manœuvre #1 or manœuvre #2” or something along
        // these lines).
        UpdateStatus(status, index);
      }
    }
    return false;
  }

  internal static string FormatPositiveTimeSpan(double seconds) {
    return new PrincipiaTimeSpan(seconds).FormatPositive(
        with_leading_zeroes: true,
        with_seconds: true);
  }

  internal static string FormatTimeSpan (double seconds) {
    return new PrincipiaTimeSpan(seconds).Format(
        with_leading_zeroes: true,
        with_seconds: true);
  }

  internal string FormatPlanLength(double value) {
    return FormatPositiveTimeSpan(value -
                                  plugin.FlightPlanGetInitialTime(
                                      predicted_vessel.id.ToString()));
  }

  internal bool TryParsePlanLength(string text, out double value) {
    value = 0;
    if (!PrincipiaTimeSpan.TryParse(text,
                                    out PrincipiaTimeSpan ts)) {
      return false;
    }
    value = ts.total_seconds +
            plugin.FlightPlanGetInitialTime(predicted_vessel.id.ToString());
    return true;
  }

  // Called to rebuild the optimizer based on the current state of the flight
  // plan and the given optimization parameters.
  private void ResetOptimizer(string vessel_guid,
                              CelestialBody centre,
                              double optimization_altitude,
                              double? optimization_inclination_in_degrees) {
    optimization_altitude_ = optimization_altitude;
    optimization_inclination_in_degrees_ = optimization_inclination_in_degrees;
    optimization_reference_frame_parameters_ =
        (NavigationFrameParameters)adapter_.plotting_frame_selector_.
            FrameParameters();
    plugin.FlightPlanOptimizationDriverMake(vessel_guid,
                                            centre.Radius +
                                            optimization_altitude_,
                                            optimization_inclination_in_degrees_,
                                            centre.flightGlobalsIndex,
                                            optimization_reference_frame_parameters_);
  }

  // Must be called each time the flight plan is changed by the user: the
  // optimizer holds a copy of the flight plan, so proceeding with it would be
  // wrong or confusing.
  private void ResetOptimizer(string vessel_guid) {
    // Rebuild the optimizer as whatever it was doing is now irrelevant.
    if (adapter_.plotting_frame_selector_.
            Centre() is CelestialBody centre) {
      ResetOptimizer(vessel_guid,
                     centre,
                     optimization_altitude_,
                     optimization_inclination_in_degrees_);
    }
  }

  private void ResetStatus() {
    status_ = Status.OK;
    first_error_manœuvre_ = null;
    message_was_displayed_ = false;
  }

  private void UpdateStatus(Status status, int? error_manœuvre) {
    if (message_was_displayed_) {
      ResetStatus();
    }
    if (status_.ok() && !status.ok()) {
      status_ = status;
      first_error_manœuvre_ = error_manœuvre;
    }
  }

  private string GetStatusMessage() {
    string vessel_guid = predicted_vessel?.id.ToString();
    string message = "";
    if (vessel_guid != null && !status_.ok()) {
      int anomalous_manœuvres =
          plugin.FlightPlanNumberOfAnomalousManoeuvres(vessel_guid);
      int manœuvres = plugin.FlightPlanNumberOfManoeuvres(vessel_guid);
      double actual_final_time =
          plugin.FlightPlanGetActualFinalTime(vessel_guid);
      bool timed_out = actual_final_time < final_time_.value;

      string remedy_message =
          L10N.CacheFormat(
              "#Principia_FlightPlan_StatusMessage_ChangeFlightPlan");  // Preceded by "Try".
      string status_message = L10N.CacheFormat(
          "#Principia_FlightPlan_StatusMessage_FailedError",
          status_.error,
          status_.message);
      string time_out_message =
          timed_out ? L10N.CacheFormat(
                          "#Principia_FlightPlan_StatusMessage_TimeOut",
                          FormatPositiveTimeSpan(
                              actual_final_time -
                              plugin.FlightPlanGetInitialTime(vessel_guid)))
                    : "";
      if (status_.is_aborted() || status_.is_resource_exhausted()) {
        status_message = L10N.CacheFormat(
            "#Principia_FlightPlan_StatusMessage_MaxSteps",
            time_out_message);
        remedy_message =
            L10N.CacheFormat("#Principia_FlightPlan_StatusMessage_MaxSegment");
      } else if (status_.is_failed_precondition()) {
        status_message = L10N.CacheFormat(
            "#Principia_FlightPlan_StatusMessage_Singularity",
            time_out_message);
        remedy_message =
            L10N.CacheFormat(
                "#Principia_FlightPlan_StatusMessage_AvoidingCollision");
      } else if (status_.is_invalid_argument()) {
        status_message = L10N.CacheFormat(
            "#Principia_FlightPlan_StatusMessage_Infinite",
            first_error_manœuvre_.Value + 1);
        remedy_message = L10N.CacheFormat(
            "#Principia_FlightPlan_StatusMessage_Adjust",
            first_error_manœuvre_.Value + 1);
      } else if (status_.is_out_of_range()) {
        if (first_error_manœuvre_.HasValue) {
          status_message = L10N.CacheFormat(
              "#Principia_FlightPlan_StatusMessage_OutRange1",
              first_error_manœuvre_.Value + 1,
              first_error_manœuvre_.Value == 0
                  ? L10N.CacheFormat(
                      "#Principia_FlightPlan_StatusMessage_OutRange2")
                  : L10N.CacheFormat(
                      "#Principia_FlightPlan_StatusMessage_OutRange3",
                      first_error_manœuvre_.Value),
              manœuvres == 0 || first_error_manœuvre_.Value == manœuvres - 1
                  ? L10N.CacheFormat(
                      "#Principia_FlightPlan_StatusMessage_OutRange4")
                  : L10N.CacheFormat(
                      "#Principia_FlightPlan_StatusMessage_OutRange5",
                      first_error_manœuvre_.Value + 2));
          remedy_message =  L10N.CacheFormat(
              "#Principia_FlightPlan_StatusMessage_OutRange6",
              manœuvres == 0 || first_error_manœuvre_.Value == manœuvres - 1
                  ? L10N.CacheFormat(
                      "#Principia_FlightPlan_StatusMessage_OutRange7")
                  : "",
              first_error_manœuvre_.Value + 1);
        } else {
          status_message =
              L10N.CacheFormat("#Principia_FlightPlan_StatusMessage_TooShort");
          remedy_message =
              L10N.CacheFormat("#Principia_FlightPlan_StatusMessage_Increase");
        }
      } else if (status_.is_deadline_exceeded()) {
        status_message =
            L10N.CacheFormat(
                "#Principia_FlightPlan_StatusMessage_DeadlineExceeded");
        remedy_message =
            L10N.CacheFormat("#Principia_FlightPlan_StatusMessage_Tickle");
      } else if (status_.is_unavailable()) {
        status_message =
            L10N.CacheFormat("#Principia_FlightPlan_StatusMessage_CantRebase");
        remedy_message =
            L10N.CacheFormat("#Principia_FlightPlan_StatusMessage_WaitFinish");
      }

      if (anomalous_manœuvres > 0) {
        message = L10N.CacheFormat(
            "#Principia_FlightPlan_StatusMessage_Last",
            anomalous_manœuvres,
            status_message ,
            remedy_message,
            (anomalous_manœuvres < manœuvres
                 ? L10N.CacheFormat("#Principia_FlightPlan_StatusMessage_Last2",
                                    manœuvres - anomalous_manœuvres)
                 : ""));
      } else {
        message =
            L10N.CacheFormat("#Principia_FlightPlan_StatusMessage_Result",
                             status_message,
                             remedy_message);
      }
    }
    message_was_displayed_ = true;
    return message;
  }

  private void UpdateBurnEditorIndices(string vessel_guid) {
    // Adjust the indices of the current burn editors.
    for (int j = 0; j < burn_editors_.Count; ++j) {
      burn_editors_[j].index = j;
    }
  }

  private void UpdateAdaptiveStepParameters(
      ref FlightPlanAdaptiveStepParameters parameters) {
    parameters.length_integration_tolerance =
        integration_tolerances_[length_integration_tolerance_index_];
    parameters.speed_integration_tolerance =
        integration_tolerances_[speed_integration_tolerance_index_];
    parameters.max_steps = max_steps_[max_steps_index_];
  }

  private IntPtr plugin => adapter_.Plugin();

  private static readonly double[] integration_tolerances_ =
      {1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6};
  private static readonly string[] length_integration_tolerances_names_ = {
      "1 µm", "10 µm", "100 µm", "1 mm", "1 cm", "10 cm", "1 m", "10 m",
      "100 m", "1 km", "10 km", "100 km", "1000 km"
  };
  private static readonly long[] max_steps_ = {
      1 << 6, 1 << 8, 1 << 10, 1 << 12, 1 << 14, 1 << 16, 1 << 18, 1 << 20
  };

  private readonly PrincipiaPluginAdapter adapter_;
  private readonly PredictedVessel predicted_vessel_;

  // Because this class is stateful (it holds the burn_editors_) we must detect
  // if the vessel changed.  Hence the caching of the vessel.
  private Vessel previous_predicted_vessel_;

  private List<BurnEditor> burn_editors_;
  private PlannedOrbitAnalyser final_trajectory_analyser_;
  private readonly DifferentialSlider final_time_;
  private int? first_future_manœuvre_;
  private int number_of_anomalous_manœuvres_ = 0;
  private bool reached_deadline_ = false;
  private bool must_tickle_ = false;

  private int length_integration_tolerance_index_;
  private int speed_integration_tolerance_index_;
  private int max_steps_index_;
  private bool show_guidance_ = false;
  private float warning_height_ = 1;

  private Status status_ = Status.OK;
  private int? first_error_manœuvre_;  // May exceed the number of manœuvres.
  private bool message_was_displayed_ = false;

  private int? requested_editor_focus_index_;

  private const double log10_time_lower_rate = 0.0;
  private const double log10_time_upper_rate = 7.0;

  private const int max_flight_plans = 10;

  private const double default_optimization_altitude = 10e3;
  private const double default_optimization_inclination_in_degrees = 0;

  private double optimization_altitude_ = default_optimization_altitude;
  private double? optimization_inclination_in_degrees_ =
      default_optimization_inclination_in_degrees;
  private NavigationFrameParameters optimization_reference_frame_parameters_;

  private enum FlightPlannerTab {
    Planner,
    Optimizer,
    Transfer,
    Timeline,
    ImportExport,
  }

  private struct TransferEstimate {
    public double wait_time;
    public double transfer_time;
    public double ejection_Δv;
    public double capture_Δv;
  }

  private FlightPlannerTab selected_tab_ = FlightPlannerTab.Planner;
  private int selected_optimization_manœuvre_ = 0;

  private CelestialBody transfer_origin_;
  private CelestialBody transfer_target_;
  private string transfer_summary_ = "";

  private bool timeline_initialized_ = false;
  private double timeline_time_ = double.NaN;

  private readonly string plan_exchange_directory_ =
      Path.Combine(KSPUtil.ApplicationRootPath,
                   "GameData",
                   "Principia",
                   "FlightPlans");
  private List<string> available_plan_files_ = new List<string>();
  private int selected_plan_file_index_ = 0;
  private string io_status_message_ = "";
}

}  // namespace ksp_plugin_adapter
}  // namespace principia
